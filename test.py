import cv2
import numpy as np
import tensorflow as tf
import tensorflow_hub as hub
from newMotor import Drive

LOW_SPEED = 0.3
AVG_SPEED = 0.5
FAST_SPEED = 0.7
MAX_SPEED = 1.0
MIN_SPEED = 0.0
isMoving = True
MAX_DISTANCE = 50
#Create drive instance
drive = Drive()
# Load lightweight SSD Mobilenet v2 model (320x320) from TF Hub
detector = hub.load("https://tfhub.dev/tensorflow/ssd_mobilenet_v2/fpnlite_320x320/1")

FOCAL_LENGTH = 750
AVERAGE_HEIGHT = 22
Kp = 0.2

def calculate_distance(known_height, focal_length, perceived_height):
    return (known_height * focal_length) / perceived_height

def follow(center, person, distance):
   print(center)
   center_y = center[1]
   error = person[1] - center_y
   turn_factor = Kp * error
   
   if (distance <= MAX_DISTANCE): #stop robot, keep human safe
      drive.stop_motors()
      isMoving = False


   elif (person[1] <= center_y + 25) and (person[1] > center_y - 25): #drive forward if in middle of frame
      drive.forward(LOW_SPEED)
      isMoving = True
  
   elif (person[1] < center_y - 25): #if person is in the left side of the camera
      speed = min(AVG_SPEED, max(MIN_SPEED, LOW_SPEED + turn_factor))
      drive.right_forward.value = speed
      speed = max(AVG_SPEED, min(AVG_SPEED, LOW_SPEED - turn_factor))
      drive.left_forward.value = speed
      isMoving = True

   elif (person[1] > center_y + 25): #if person is in the right side of the camera
      speed = max(MIN_SPEED, min(AVG_SPEED, LOW_SPEED - turn_factor))
      drive.left_forward.value = speed
      speed = min(AVG_SPEED, max(MIN_SPEED, LOW_SPEED + turn_factor))
      drive.right_forward.value = speed
      isMoving = True
   

# Start webcam
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret:
        break

    height, width, _ = frame.shape
    center = (width//2, height//2)
   
    # Preprocess frame for the model
    resized_frame = cv2.resize(frame, (320, 320))  # Resize image to 320x320
    input_tensor = tf.convert_to_tensor([resized_frame], dtype=tf.uint8)  # Convert to uint8
    #input_tensor = tf.image.convert_image_dtype(input_tensor, tf.uint8)  # Normalize to [0, 1]

    # Run detection
    detections = detector(input_tensor)

    # Get detection results
    boxes = detections['detection_boxes'][0].numpy()
    classes = detections['detection_classes'][0].numpy().astype(np.int32)
    scores = detections['detection_scores'][0].numpy()
    
    for i in range(len(scores)):

        if classes[i] != 1:  #if no human detected, spin in circle
            drive.circle_around()        
        if scores[i] > 0.5 and classes[i] == 1:  # Class 1 = person in COCO dataset
            y_min, x_min, y_max, x_max = boxes[i]
            xA, yA = int(x_min * width), int(y_min * height)
            xB, yB = int(x_max * width), int(y_max * height)

            # Draw bounding box around detected person
   #         cv2.rectangle(frame, (xA, yA), (xB, yB), (0, 255, 0), 2)

            # Estimate distance
            perceived_height = xB - xA  # Height of the detected person in pixels
            print("Perceived height: ", perceived_height)
            if perceived_height > 0:
                distance = calculate_distance(AVERAGE_HEIGHT, FOCAL_LENGTH, perceived_height)
                print("DISTANCE: ", distance)
               # cv2.putText(frame, f"Distance: {distance:.2f} cm", (xA, yA - 10),
                #            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
               
                #Drive logic: Stop if near human, spin to look for human, follow based on human's coords in frame
                follow(center, (((xB-xA)//2), ((yB-yA)//2)), 120)                
                print("Person Center Coord: ", (((xB-xA)//2), ((yB-yA)//2)))
           
                if (isMoving == False and distance >= MAX_DISTANCE):
                  isMoving = True
                  drive.circle_around()

    # Show the resulting frame with detection and distance
  #  cv2.imshow("Detection and Distance", frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
