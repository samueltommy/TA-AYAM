import torch
from ultralytics import YOLO
import cv2
import numpy as np
from pathlib import Path
import time
import argparse
from sklearn.preprocessing import StandardScaler
import joblib
import requests
from datetime import datetime
import os
from collections import deque
import tkinter as tk
import platform
import subprocess

# Firebase config
FIREBASE_API_KEY = ""
FIREBASE_EMAIL = ""
FIREBASE_PASSWORD = ""
DATABASE_URL = ""

# Firebase authentication globals
firebase_id_token = None
firebase_token_expiry = 0
firebase_refresh_token = None

def firebase_authenticate():
    """Authenticate with Firebase and get idToken."""
    global firebase_id_token, firebase_token_expiry, firebase_refresh_token
    url = f"https://identitytoolkit.googleapis.com/v1/accounts:signInWithPassword?key={FIREBASE_API_KEY}"
    payload = {
        "email": FIREBASE_EMAIL,
        "password": FIREBASE_PASSWORD,
        "returnSecureToken": True
    }
    resp = requests.post(url, json=payload)
    if resp.status_code == 200:
        data = resp.json()
        firebase_id_token = data.get("idToken")
        firebase_refresh_token = data.get("refreshToken")
        expires_in = int(data.get("expiresIn", 3600))
        firebase_token_expiry = time.time() + expires_in - 60
        print("[Firebase] Authenticated successfully.")
    else:
        print(f"[Firebase] Authentication failed: {resp.text}")
        firebase_id_token = None

def get_firebase_auth_param():
    """Return ?auth=... for Firebase requests, refreshing if needed."""
    global firebase_id_token, firebase_token_expiry
    if not firebase_id_token or time.time() > firebase_token_expiry:
        firebase_authenticate()
    return f"?auth={firebase_id_token}" if firebase_id_token else ""

def is_display_available():
    # Force headless mode if environment variable is set
    if os.environ.get('HEADLESS', '0') == '1':
        return False
    # Try to create a window and catch errors
    try:
        cv2.namedWindow('test')
        cv2.destroyWindow('test')
        return True
    except Exception:
        return False

class ChickenTracker:
    """Simple centroid-based tracker for chicken IDs across frames."""
    def __init__(self, max_lost=10):
        self.next_id = 0
        self.objects = dict()  # id -> {'centroid': (x, y), 'lost': 0}
        self.max_lost = max_lost

    def update(self, detections):
        # detections: list of {'bbox': [x1, y1, x2, y2], ...}
        centroids = []
        for d in detections:
            x1, y1, x2, y2 = d['bbox']
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            centroids.append((cx, cy))
        # Match existing objects to new centroids
        assigned = set()
        new_objects = dict()
        for obj_id, obj in self.objects.items():
            min_dist = float('inf')
            min_idx = -1
            for i, c in enumerate(centroids):
                if i in assigned:
                    continue
                dist = np.linalg.norm(np.array(obj['centroid']) - np.array(c))
                if dist < min_dist:
                    min_dist = dist
                    min_idx = i
            if min_dist < 50 and min_idx != -1:  # threshold for matching
                new_objects[obj_id] = {'centroid': centroids[min_idx], 'lost': 0}
                assigned.add(min_idx)
            else:
                # Not matched, increase lost
                if obj['lost'] + 1 < self.max_lost:
                    new_objects[obj_id] = {'centroid': obj['centroid'], 'lost': obj['lost'] + 1}
        # Add new objects
        for i, c in enumerate(centroids):
            if i not in assigned:
                new_objects[self.next_id] = {'centroid': c, 'lost': 0}
                self.next_id += 1
        self.objects = new_objects
        # Return mapping: idx in detections -> id
        idx_to_id = {}
        for idx, c in enumerate(centroids):
            for obj_id, obj in self.objects.items():
                if np.linalg.norm(np.array(obj['centroid']) - np.array(c)) < 1e-3:
                    idx_to_id[idx] = obj_id
        return idx_to_id

class ChickenDetector:
    def __init__(self, model_path='yolo-result/best.pt', imgsz=640):
        """Initialize the chicken detector with trained YOLO model."""
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f"Using device: {self.device}")
        
        # Load YOLO model with optimization
        print("Loading YOLO model...")
        self.model = YOLO(model_path)
        # Improved for small/clustered chicks
        self.model.conf = 0.4  # Lower confidence threshold
        self.model.iou = 0.3   # Lower NMS IoU threshold
        self.model.agnostic = False
        self.model.max_det = 20  # Allow more detections per image
        self.imgsz = 1280  # Use higher input resolution for small objects  # Use default 640, can be tuned
        print(f"Model loaded successfully! Using imgsz={self.imgsz}")
        
        # Initialize weight prediction components
        self.scaler = StandardScaler()
        self.area_to_weight_factor = 85000

        # Expanded calibration data (real_weight: predicted_weight)
        self.calibration_data = {
            0.100: 0.05,
            0.150: 0.076,   
            0.289: 0.15,   
            0.378: 0.19,   
            0.345: 0.18,   
            0.405: 0.21,   
            0.420: 0.25,   
            0.450: 0.26,  
            0.483: 0.29,  
            0.540: 0.30,   
            0.600: 0.302,  
            0.650: 0.304,   
            0.715: 0.306,
            0.790: 0.309,
            0.82: 0.313,
            0.85: 0.316,

        }
        real_weights = np.array(list(self.calibration_data.keys()))
        pred_weights = np.array(list(self.calibration_data.values()))
        # Quadratic regression: fit 2nd degree polynomial
        self.poly_coeffs = np.polyfit(pred_weights, real_weights, 2)
        print(f"Calibration polynomial: real = {self.poly_coeffs[0]:.4f}*pred^2 + {self.poly_coeffs[1]:.4f}*pred + {self.poly_coeffs[2]:.4f}")
        print(f"YOLO input resolution set to {self.imgsz}")

    def is_inside_optimum_spot(self, bbox, frame_width, frame_height):
        """Check if the bounding box center lies within the optimum spot."""
        x1, y1, x2, y2 = bbox
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        roi_w = frame_width // 2         # 1/2 width
        roi_h = (3 * frame_height) // 4  # 3/4 height
        roi_x1 = (frame_width - roi_w) // 2
        roi_y1 = frame_height - roi_h
        roi_x2 = roi_x1 + roi_w
        roi_y2 = roi_y1 + roi_h

        return roi_x1 <= cx <= roi_x2 and roi_y1 <= cy <= roi_y2
        
    def preprocess_frame(self, frame):
        """Preprocess frame: always resize to 1280x720 (letterbox if needed)."""
        target_w, target_h = 1280, 720
        h, w = frame.shape[:2]
        scale = min(target_w / w, target_h / h)
        new_w, new_h = int(w * scale), int(h * scale)
        resized = cv2.resize(frame, (new_w, new_h))
        # Create a black canvas and paste the resized frame (letterbox)
        canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)
        x_offset = (target_w - new_w) // 2
        y_offset = (target_h - new_h) // 2
        canvas[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized
        return canvas

    def calculate_weight(self, mask):
        """Calculate weight based on segmentation mask area and apply improved calibration."""
        if mask is None:
            return 0.0, 0
        
        # Count non-zero pixels in the mask to get the area
        area = np.count_nonzero(mask)
        
        # If area is too small, it's likely not a valid detection
        if area < 100:
            return 0.0, area
            
        # Use area to get initial predicted weight (proportional to area)
        base_pred_weight = 1.920 * (area / 71667)
        # Apply quadratic calibration
        a, b, c = self.poly_coeffs
        calibrated_weight = a * base_pred_weight**2 + b * base_pred_weight + c
        
        return calibrated_weight, area

    def detect_chicken(self, frame):
        """Detect chicken and predict weight in a single frame using only mask area."""
        results = self.model(frame, imgsz=self.imgsz, verbose=False)[0]
        
        # Check if any detection is available
        if len(results.boxes) == 0 or (hasattr(results, 'masks') and len(results.masks) == 0):
            return frame, None, 0.0, 0.0, 0
            
        # Get best confidence detection
        conf = float(results.boxes.conf[0])
        
        # Create mask from segmentation
        if hasattr(results, 'masks') and len(results.masks) > 0:
            # Use segmentation mask
            mask = results.masks.data[0].cpu().numpy()
            binary_mask = (mask > 0.5).astype(np.uint8)
            
            # Resize mask to match frame dimensions if needed
            if binary_mask.shape != (frame.shape[0], frame.shape[1]):
                binary_mask = cv2.resize(binary_mask, (frame.shape[1], frame.shape[0]))
        else:
            # No segmentation mask available
            return frame, None, 0.0, 0.0, 0
            
        # Compute area and weight
        weight, area = self.calculate_weight(binary_mask)
        
        # Create a colored mask for visualization
        overlay = frame.copy()
        # Apply green color to the mask area
        overlay[binary_mask == 1] = (0, 255, 0)  
        frame_with_mask = cv2.addWeighted(overlay, 0.3, frame, 0.7, 0)
        
        # Draw text information
        cv2.putText(frame_with_mask, f'Weight: {weight:.3f}kg', 
                  (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                  1, (255, 255, 255), 2)
        cv2.putText(frame_with_mask, f'Area: {area} px', 
                  (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 
                  1, (255, 255, 255), 2)
        cv2.putText(frame_with_mask, f'Conf: {conf:.2f}', 
                  (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 
                  1, (255, 255, 255), 2)
                  
        # For tracking purposes only, still get the bbox
        if hasattr(results, 'boxes') and len(results.boxes) > 0:
            bbox = results.boxes.xyxy[0].cpu().numpy().astype(int)
        else:
            bbox = None
            
        return frame_with_mask, bbox, conf, weight, area

    def detect_chickens(self, frame):
        """Detect multiple chickens and predict weights in a single frame."""
        results = self.model(frame, imgsz=self.imgsz, verbose=False)[0]
        chickens = []
        if len(results.boxes) == 0:
            return frame, chickens
        # For each detection
        for i in range(len(results.boxes)):
            conf = float(results.boxes.conf[i])
            bbox = results.boxes.xyxy[i].cpu().numpy().astype(int)
            mask = None
            if hasattr(results, 'masks') and len(results.masks) > i:
                mask_data = results.masks.data[i].cpu().numpy()
                binary_mask = (mask_data > 0.5).astype(np.uint8)
                if binary_mask.shape != (frame.shape[0], frame.shape[1]):
                    binary_mask = cv2.resize(binary_mask, (frame.shape[1], frame.shape[0]))
                mask = binary_mask
            weight, area = self.calculate_weight(mask) if mask is not None else (0.0, 0)
            # Area filter: skip detections with very small or very large area
            if area < 2000 or area > 120000:
                continue
            # Confidence filter: skip detections below threshold (should be redundant, but double check)
            if conf < 0.5:
                continue
            chickens.append({
                'bbox': bbox,
                'conf': conf,
                'weight': weight,
                'area': area,
                'mask': mask
            })
        # Visualization
        frame_with_mask = frame.copy()
        for c in chickens:
            if c['mask'] is not None:
                overlay = frame_with_mask.copy()
                overlay[c['mask'] == 1] = (0, 255, 0)
                frame_with_mask = cv2.addWeighted(overlay, 0.3, frame_with_mask, 0.7, 0)
            if c['bbox'] is not None:
                cv2.rectangle(frame_with_mask, (c['bbox'][0], c['bbox'][1]), (c['bbox'][2], c['bbox'][3]), (0, 0, 255), 2)
                cv2.putText(frame_with_mask, f'W:{c["weight"]:.2f}kg', (c['bbox'][0], c['bbox'][1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        return frame_with_mask, chickens

    def send_to_firebase(self, chicken_data, minimal=False):
        """Send chicken weight data to Firebase Realtime Database with authentication, efficient for free tier."""
        if not chicken_data:
            return  # Don't send empty data
        # Use a single timestamped key for the batch
        try:
            base_time = chicken_data[0]['timestamp']
            batch_time = datetime.fromisoformat(base_time).strftime('%Y%m%dT%H%M%S')
        except Exception:
            batch_time = datetime.now().strftime('%Y%m%dT%H%M%S')
        url = f"{DATABASE_URL}/chicken-weight/{batch_time}.json{get_firebase_auth_param()}"
        if minimal:
            # Only send minimal fields: id, weight, area, timestamp
            payload = [
                {'id': d['id'], 'weight': d['weight'], 'area': d['area'], 'timestamp': d['timestamp']} for d in chicken_data
            ]
        else:
            # Send all fields in the dict (for summary stats)
            payload = chicken_data
        print(f"[Firebase DEBUG] URL: {url}")
        print(f"[Firebase DEBUG] Payload: {payload}")
        try:
            r = requests.put(url, json=payload)
            print(f"[Firebase DEBUG] Response: {r.status_code} {r.text}")
            if r.status_code == 200:
                print(f"[Firebase] Batch of {len(payload)} records sent successfully.")
            else:
                print(f"[Firebase] Failed to send data: {r.text}")
        except Exception as e:
            print(f"[Firebase] Exception: {e}")

    def compute_and_send_stats(self, buffer, capture_date=None):
        if not buffer:
            print("No data to send.")
            return
        weights = np.array([d['weight'] for d in buffer])
        areas = np.array([d['area'] for d in buffer])
        stats = {
            'count': len(buffer),
            'weight_median': float(np.median(weights)),
            'weight_iqr': float(np.percentile(weights, 75) - np.percentile(weights, 25)),
            'weight_min': float(np.min(weights)),
            'weight_max': float(np.max(weights)),
            'weight_mean': float(np.mean(weights)),
            'area_median': float(np.median(areas)),
            'area_iqr': float(np.percentile(areas, 75) - np.percentile(areas, 25)),
            'area_min': float(np.min(areas)),
            'area_max': float(np.max(areas)),
            'area_mean': float(np.mean(areas)),
            'timestamp': capture_date if capture_date else datetime.now().isoformat()
        }
        print("Sending stats to Firebase:", stats)
        self.send_to_firebase([stats], minimal=False)

    def process_video(self, video_path, video_capture_date=None):
        """Process video and send individual chicken detections to Firebase, matching RTSP logic."""
        if not Path(video_path).exists():
            print(f"Error: Video file not found at {video_path}")
            return

        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print("Error: Could not open video file")
            return

        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(cap.get(cv2.CAP_PROP_FPS))
        try:
            root = tk.Tk()
            root.withdraw()
            screen_width = root.winfo_screenwidth()
            screen_height = root.winfo_screenheight()
            root.destroy()
        except Exception:
            screen_width, screen_height = 640, 480
        scale = min(screen_width / frame_width, screen_height / frame_height, 1.0)
        out_w, out_h = int(frame_width * scale), int(frame_height * scale)

        output_path = Path(video_path).with_name(f"{Path(video_path).stem}_detected.mp4")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(str(output_path), fourcc, fps, (out_w, out_h))

        live_data_path = Path(video_path).with_name(f"{Path(video_path).stem}_live_data.csv")
        with open(live_data_path, 'w') as live_file:
            live_file.write("Frame,ChickenID,Weight,Area,Confidence\n")

        print(f"Video properties: {frame_width}x{frame_height} @ {fps}fps")
        print(f"Processing video... Output will be saved to {output_path}")
        print(f"Live detection data will be saved to {live_data_path}")

        frame_times = []
        frame_count = 0
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        chicken_max_weights = dict()
        firebase_buffer = []
        stats_buffer = []
        tracker = ChickenTracker()
        prev_centroids = {}
        speed_threshold = 20
        display_size = (out_w, out_h)
        while cap.isOpened():
            start_time = time.time()
            ret, frame = cap.read()
            frame_start_time = time.time()
            if not ret:
                print("\nEnd of video reached")
                break
            resized_frame = self.preprocess_frame(frame)
            frame_with_mask, chickens = self.detect_chickens(resized_frame)
            # Draw optimum spot ROI in blue
            roi_w = frame_with_mask.shape[1] // 2
            roi_h = (3 * frame_with_mask.shape[0]) // 4
            roi_x1 = (frame_with_mask.shape[1] - roi_w) // 2
            roi_y1 = frame_with_mask.shape[0] - roi_h
            roi_x2 = roi_x1 + roi_w
            roi_y2 = roi_y1 + roi_h
            cv2.rectangle(frame_with_mask, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 255, 0), 2)
            cv2.putText(frame_with_mask, "OPTIMUM SPOT", (roi_x1 + 10, roi_y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            idx_to_id = tracker.update(chickens)
            chicken_data = []
            now_time = time.time()
            for idx, c in enumerate(chickens):
                chicken_id = f"chicken_{idx_to_id.get(idx, idx)}"
                bbox = c['bbox']
                cx = int((bbox[0] + bbox[2]) / 2)
                cy = int((bbox[1] + bbox[3]) / 2)
                prev = prev_centroids.get(chicken_id)
                speed = 0
                if prev:
                    prev_cx, prev_cy, prev_time = prev
                    dist = np.linalg.norm([cx - prev_cx, cy - prev_cy])
                    dt = now_time - prev_time
                    if dt > 0:
                        speed = dist / dt
                prev_centroids[chicken_id] = (cx, cy, now_time)
                if speed > speed_threshold:
                    continue
                prev_max = chicken_max_weights.get(chicken_id, 0)
                if c['weight'] > prev_max:
                    chicken_max_weights[chicken_id] = c['weight']
                if video_capture_date is not None:
                    ts = video_capture_date if isinstance(video_capture_date, str) else video_capture_date.isoformat()
                else:
                    ts = datetime.now().isoformat()
                if self.is_inside_optimum_spot(bbox, frame.shape[1], frame.shape[0]):
                    data = {
                        'id': chicken_id,
                        'weight': c['weight'],
                        'area': c['area'],
                        'conf': c['conf'],
                        'timestamp': ts,
                        'speed': speed
                    }
                    chicken_data.append(data)
                    stats_buffer.append(data)
                    with open(live_data_path, 'a') as live_file:
                        live_file.write(f"{frame_count},{chicken_id},{c['weight']:.3f},{c['area']},{c['conf']:.3f}\n")
            # Buffer and send to Firebase in batches (up to 5 unique chickens)
            firebase_buffer.extend(chicken_data)
            id_seen = set()
            filtered_buffer = []
            for d in reversed(firebase_buffer):
                if d['id'] not in id_seen:
                    filtered_buffer.append(d)
                    id_seen.add(d['id'])
                if len(id_seen) >= 5:
                    break
            firebase_buffer = list(reversed(filtered_buffer))
            if chicken_data:
                self.send_to_firebase(chicken_data, minimal=True)
            # Overlay all detected chickens (already done in detect_chickens)
            # FPS and progress
            frame_time = time.time() - start_time
            frame_times.append(frame_time)
            if len(frame_times) > 30:
                frame_times.pop(0)
            fps_current = 1.0 / (sum(frame_times) / len(frame_times))
            cv2.putText(frame_with_mask, f'FPS: {fps_current:.1f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            display_frame = cv2.resize(frame_with_mask, display_size)
            cv2.imshow('Chicken Detection (Video)', display_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n[Info] Quit by user.")
                break
            frame_end_time = time.time()
            processing_time = frame_end_time - frame_start_time
            print(f"\n[Profiling] Frame {frame_count} processing time: {processing_time:.2f} s")
            out.write(display_frame)
            frame_count += 1
            progress = (frame_count / total_frames) * 100
            print(f"\rProcessing: {progress:.1f}% | Chickens: {len(chickens)}", end="")
        cap.release()
        out.release()
        cv2.destroyAllWindows()
        print("\nAnalysis complete!")
        print(f"Results saved to: {output_path}")
        print(f"Live detection data saved to: {live_data_path}")
        # Send summary statistics to Firebase after video analysis
        if stats_buffer:
            self.compute_and_send_stats(stats_buffer, capture_date=video_capture_date)
        print("Most representative (max) weights per chicken:")
        for cid, w in chicken_max_weights.items():
            print(f"{cid}: {w:.3f} kg")
        
    def get_display_size(self, frame):
        try:
            root = tk.Tk()
            root.withdraw()
            screen_width = root.winfo_screenwidth()
            screen_height = root.winfo_screenheight()
            root.destroy()
        except Exception:
            screen_width, screen_height = 640, 480  # fallback
        h, w = frame.shape[:2]
        scale = min(screen_width / w, screen_height / h, 1.0)
        new_w, new_h = int(w * scale), int(h * scale)
        return new_w, new_h

    def is_within_capture_window(self):
        now = datetime.now().time()
        return (now.hour == 7 or now.hour == 8 or now.hour == 19 or now.hour == 20)

    def get_video_capture_date(video_path):
        """Extract the original capture date from video file metadata using ffprobe. Returns ISO string or None."""
        try:
            cmd = [
                'ffprobe',
                '-v', 'error',
                '-select_streams', 'v:0',
                '-show_entries', 'format_tags=creation_time',
                '-of', 'default=noprint_wrappers=1:nokey=1',
                video_path
            ]
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            date_str = result.stdout.strip()
            if date_str:
                # Try to parse to datetime and return ISO format
                try:
                    dt = datetime.fromisoformat(date_str.replace('Z', '+00:00'))
                    return dt.isoformat()
                except Exception:
                    return date_str  # fallback: raw string
            else:
                return None
        except Exception as e:
            print(f"[Warning] Could not extract video capture date: {e}")
            return None

    def process_stream(self, source, view_only=False, video_capture_date=None):
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            print(f"Error: Could not open source {source}")
            return
        print(f"Processing stream: {source}")
        frame_count = 0
        chicken_max_weights = dict()
        firebase_buffer = []  # Buffer for chicken data
        stats_buffer = []     # Buffer for all detections for stats
        tracker = ChickenTracker()
        prev_centroids = {}
        speed_threshold = 20
        def is_stats_time():
            now = datetime.now().time()
            return (now.hour == 9 or now.hour == 21) and (0 <= now.minute < 10)
        last_stats_sent = None
        display_size = None
        while True:
            start_time = time.time()
            ret, frame = cap.read()
            if not ret:
                print("\nStream ended or cannot fetch frame.")
                break
            elapsed = time.time() - start_time
            if elapsed > 2.0:
                print(f"[Warning] Frame grab took {elapsed:.1f}s, skipping frame.")
                continue
            resized_frame = self.preprocess_frame(frame)
            frame_with_mask, chickens = self.detect_chickens(resized_frame)
            # Draw optimum spot ROI in blue
            roi_w = frame_with_mask.shape[1] // 2
            roi_h = (3 * frame_with_mask.shape[0]) // 4
            roi_x1 = (frame_with_mask.shape[1] - roi_w) // 2
            roi_y1 = frame_with_mask.shape[0] - roi_h
            roi_x2 = roi_x1 + roi_w
            roi_y2 = roi_y1 + roi_h
            cv2.rectangle(frame_with_mask, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 255, 0), 2)
            cv2.putText(frame_with_mask, "OPTIMUM SPOT", (roi_x1 + 10, roi_y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            idx_to_id = tracker.update(chickens)
            chicken_data = []
            now_time = time.time()
            # Only process and buffer detections in capture window (unless view_only)
            if self.is_within_capture_window() or view_only:
                for idx, c in enumerate(chickens):
                    chicken_id = f"chicken_{idx_to_id.get(idx, idx)}"
                    bbox = c['bbox']
                    cx = int((bbox[0] + bbox[2]) / 2)
                    cy = int((bbox[1] + bbox[3]) / 2)
                    prev = prev_centroids.get(chicken_id)
                    speed = 0
                    if prev:
                        prev_cx, prev_cy, prev_time = prev
                        dist = np.linalg.norm([cx - prev_cx, cy - prev_cy])
                        dt = now_time - prev_time
                        if dt > 0:
                            speed = dist / dt
                    prev_centroids[chicken_id] = (cx, cy, now_time)
                    if speed > speed_threshold:
                        continue
                    prev_max = chicken_max_weights.get(chicken_id, 0)
                    if c['weight'] > prev_max:
                        chicken_max_weights[chicken_id] = c['weight']
                    # Use video_capture_date if provided, else current time
                    if video_capture_date is not None:
                        ts = video_capture_date if isinstance(video_capture_date, str) else video_capture_date.isoformat()
                    else:
                        ts = datetime.now().isoformat()
                    data = {
                        'id': chicken_id,
                        'weight': c['weight'],
                        'area': c['area'],
                        'conf': c['conf'],
                        'timestamp': ts,
                        'speed': speed
                    }
                    chicken_data.append(data)
                    stats_buffer.append(data)
                if not view_only:
                    firebase_buffer.extend(chicken_data)
                    id_seen = set()
                    filtered_buffer = []
                    for d in reversed(firebase_buffer):
                        if d['id'] not in id_seen:
                            filtered_buffer.append(d)
                            id_seen.add(d['id'])
                        if len(id_seen) >= 5:
                            break
                    firebase_buffer = list(reversed(filtered_buffer))
                # Scheduled stats sending at 9AM/9PM (first 10 min)
                if not view_only and is_stats_time():
                    if last_stats_sent is None or (datetime.now().hour != last_stats_sent):
                        self.compute_and_send_stats(stats_buffer, capture_date=video_capture_date)
                        stats_buffer.clear()
                        last_stats_sent = datetime.now().hour
            # Dynamic display size
            if display_size is None:
                display_size = self.get_display_size(frame_with_mask)
            display_frame = cv2.resize(frame_with_mask, display_size)
            cv2.imshow('Chicken Detection', display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            frame_count += 1
        cap.release()
        cv2.destroyAllWindows()
        if not view_only and stats_buffer:
            print(f"\n[Firebase] Final stats send of {len(stats_buffer)} records...")
            self.compute_and_send_stats(stats_buffer, capture_date=video_capture_date)
        print("\nMost representative (max) weights per chicken:")
        for cid, w in chicken_max_weights.items():
            print(f"{cid}: {w:.3f} kg")
        summary = [ {'id': cid, 'max_weight': w} for cid, w in chicken_max_weights.items() ]
        if not view_only:
            self.send_to_firebase(summary)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--view-only', action='store_true', help='View detections in real time without saving/sending data')
    args = parser.parse_args()
    firebase_authenticate()
    
    source = 'rtsp://taayam:taayam@192.168.1.200:554/stream2'
    print(f"Using fixed RTSP URL: {source}")
    video_capture_date = None

    # Automatically set model path
    model_path = 'yolo-result/best.pt'
    if not Path(model_path).exists():
        print(f"Error: Model file not found at {model_path}")
        return
    
    detector = ChickenDetector(str(model_path))
    print("Starting detection. If you do not see a video window, check your display environment.")
    detector.process_stream(source, view_only=False, video_capture_date=video_capture_date)

if __name__ == "__main__":
    main()