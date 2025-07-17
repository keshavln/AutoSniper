import threading
import time
import cv2
import supervision as sv
from supervision import ByteTrack
from ultralytics import YOLO
import serial

firstrun = False
detected = False
prev_bbox = None
alpha = 0.8
centerx = 320
centery = 240
targetx, targety = 320,240

esp = serial.Serial('COM3', 115200, timeout=1)
time.sleep(2)

model = YOLO(r"runs\detect\train3\weights\best.pt")

box_annotator = sv.BoxAnnotator()
label_annotator = sv.LabelAnnotator()

tracker = ByteTrack()

lock = threading.Lock()

# ---------- MOTOR CONTROL ----------
def motor_control_loop():
    global centerx, centery
    buffer = 0
    while True:
        with lock:
        # Moves motors
          if not(abs(centerx-targetx) < 50 and abs(centery-targety) < 50 and firstrun):
            buffer = 0
            if centerx < targetx:
                esp.write(b'RIGHT\n')
            if centerx > targetx:
                esp.write(b'LEFT\n')
            if centery < targety:
                esp.write(b'UP\n')
            if centery > targety:
                esp.write(b'DOWN\n')
          elif detected:
            buffer += 1
        if buffer > 3:
            esp.write(b'SHOOT\n')
            break
        # Waiting logic
        if abs(centerx-targetx) < 75 and abs(centery-targety) < 75:
            for x in range(100):
                time.sleep(0.01)
                if abs(centerx-targetx) > 75 and abs(centery-targety) > 75:
                    break

        else:
            time.sleep(0.05)



motor_thread = threading.Thread(target=motor_control_loop, daemon=True)
motor_thread.start()

# ---------- COMPUTER VISION ----------

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    frame = cv2.flip(frame,1)
    if not ret:
        continue

    results = model(frame, conf=0.5)[0]

    detections = sv.Detections.from_ultralytics(results)

    tracked_detections = tracker.update_with_detections(detections)

    # Considering detection with highest confidence + smoothing
    if len(tracked_detections.confidence) > 0:
        detected = True
        max_idx = tracked_detections.confidence.argmax()
        tracked_detections = tracked_detections[max_idx:max_idx+1]
        x1, y1, x2, y2 = tracked_detections.xyxy[0]
        if prev_bbox is None:
            smooth_bbox = (x1, y1, x2, y2)
        else:
            smooth_bbox = tuple(
                alpha * prev + (1 - alpha) * curr
                for prev, curr in zip(prev_bbox, (x1, y1, x2, y2))
            )
        prev_bbox = smooth_bbox

        tracked_detections.xyxy[0] = smooth_bbox

        # Calculating the center of the bounding box
        bbox = tracked_detections.xyxy[0]

        x1, y1, x2, y2 = bbox
        with lock:
            centerx = int((x1 + x2) / 2)
            centery = int((y1 + y2) / 2)

        labels = [
            f"{model.model.names[tracked_detections.class_id[0]]} {tracked_detections.confidence[0]:.2f}"
        ]

        annotated_frame = box_annotator.annotate(scene=frame.copy(), detections=tracked_detections)
        annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=tracked_detections, labels=labels)
    else:
        detected = False
        annotated_frame = frame
        with lock:
            centerx = 320
            centery = 240

    firstrun = True
    cv2.imshow("Auto Sniper", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
