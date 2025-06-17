import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import time

# Load the YOLO model
model = YOLO("capacitor_colab.pt")  # Replace with your model path

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Enable both color and depth streams
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
pipeline.start(config)

# Align depth frames to color frames
align = rs.align(rs.stream.color)

# Dictionary to store previous detection data for each object ID
object_history = {}  # track_id: {'distance': ..., 'time': ...}

def get_live_feed():
    """Function to get live feed from RealSense D435 with YOLO detection and distance/speed calculation."""
    try:
        while True:
            # Wait for frames and align them
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            
            # Get color and depth frames
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                continue
            
            # Convert frames to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Perform YOLO tracking/detection
            results = model.track(color_image, show=False, tracker="bytetrack.yaml", conf=0.5)
            
            # Get the annotated frame (includes bounding boxes and class labels)
            annotated_frame = results[0].plot()
            
            # Process detections
            if results[0].boxes is not None and len(results[0].boxes) > 0:
                for box in results[0].boxes:
                    # Get tracking ID of the object
                    track_id = int(box.id) if box.id is not None else None
                    if track_id is None:
                        continue  # Skip objects without IDs
                    
                    # Get bounding box coordinates
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # Calculate center of bounding box
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    
                    # Get depth value at center (distance in mm)
                    depth_value = depth_frame.get_distance(center_x, center_y) * 1000  # Convert to mm
                    
                    if depth_value > 0:  # Valid depth reading
                        current_time = time.time()
                        speed = None
                        
                        # If we have previous data for this object
                        if track_id in object_history:
                            prev_data = object_history[track_id]
                            time_diff = current_time - prev_data['time']
                            distance_diff = depth_value - prev_data['distance']
                            
                            if time_diff > 0:
                                speed = distance_diff / time_diff  # Speed in mm/s

                        # Update history for this object
                        object_history[track_id] = {
                            'distance': depth_value,
                            'time': current_time
                        }

                        # Display distance and speed BELOW the bounding box
                        text_y_offset = 15
                        cv2.putText(annotated_frame, f"ID {track_id} | Dist: {depth_value:.1f} mm",
                                    (x1, y2 + text_y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (0, 255, 0), 2)
                        if speed is not None:
                            cv2.putText(annotated_frame, f"Speed: {speed:.2f} mm/s",
                                        (x1, y2 + text_y_offset + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                        (255, 0, 0), 2)

            # Show live feed
            cv2.imshow('RealSense YOLO Tracking', annotated_frame)

            # Press 'q' to quit
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

# Run the live feed function
if __name__ == "__main__":
    get_live_feed()
