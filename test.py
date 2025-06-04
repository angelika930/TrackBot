import cv2
import numpy as np
import tensorflow as tf
import tensorflow_hub as hub
import time
from newMotor import Drive



LOW_SPEED = 0.3
AVG_SPEED = 0.5
FAST_SPEED = 0.7
MAX_SPEED = 1.0
MIN_SPEED = 0.0
isMoving = True
turn_direction = "none"
left_speed = 0
right_speed = 0
foundFailed = 0

MAX_DISTANCE = 150
#Create drive instance
drive = Drive()
# Load lightweight SSD Mobilenet v2 model (320x320) from TF Hub
detector = hub.load("https://tfhub.dev/tensorflow/ssd_mobilenet_v2/fpnlite_320x320/1")

FOCAL_LENGTH = 750
AVERAGE_HEIGHT = 25
Kp = 0.2

def calculate_distance(known_height, focal_length, perceived_height):
    return (known_height * focal_length) / perceived_height

def follow(center, person, distance):
   global isMoving
   global left_speed
   global right_speed
   global turn_direction
   global foundFailed

   print(center)
   center_x = center[0]
   error = person[0] - center_x
   turn_factor = Kp * error
   
   if (distance <= MAX_DISTANCE): #stop robot, keep human safe
      #drive.stop_motors()
      isMoving = False
      print("Made it through first\n")
   
   elif (person[0] <= center_x + 50) and (person[0] > center_x - 50): #drive forward if in middle of frame
      drive.forward(LOW_SPEED)
      isMoving = True
      turn_direction = "forward"
      print("Made it through 2nd\n")

   elif (person[0] < center_x - 50): #if person is in the left side of the camera
        left_speed = max(0.1, 0.2 - abs(turn_factor))
        right_speed = min(0.2, 0.2 + abs(turn_factor))
        #drive.left_forward.value = left_speed
        #drive.right_forward.value = right_speed
        isMoving = True
        turn_direction = "left"
        print("Made it through third\n")

   elif (person[0] > center_x + 50): #if person is in the right side of the camera
        right_speed = max(0.1, 0.2 - abs(turn_factor))
        left_speed = min(0.2, 0.2 + abs(turn_factor))
        #drive.left_forward.value = left_speed
        #drive.right_forward.value = right_speed
        isMoving = True
        turn_direction = "right"
        print("Made it through fourth\n")
   print("Left Speed: ", left_speed)
   print("right speed: ", right_speed)
   print("Made it through none\n") 
   
   print("Turn Factor: ", turn_factor)
# Start webcam
cap = cv2.VideoCapture(0)

personFound = False
while True:
    ret, frame = cap.read()
    if not ret:
        break

    height, width, _ = frame.shape
    center = (width//2, height//2)
   
    # Preprocess frame for the model
    resized_frame = cv2.resize(frame, (320, 320))  # Resize image to 320x320
    input_tensor = tf.convert_to_tensor([resized_frame], dtype=tf.uint8)  # Convert to uint8
    input_tensor = tf.image.convert_image_dtype(input_tensor, tf.uint8)  # Normalize to [0, 1]

    # Run detection
    detections = detector(input_tensor)

    # Get detection results
    boxes = detections['detection_boxes'][0].numpy()
    classes = detections['detection_classes'][0].numpy().astype(np.int32)
    scores = detections['detection_scores'][0].numpy()
    
    for i in range(len(scores)):

      #  if classes[i] != 1:  #if no human detected, spin in circle
       #     drive.circle_around()        
        if scores[i] > 0.7 and classes[i] == 1:  # Class 1 = person in COCO dataset
            personFound = True
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
                follow(center, (((xB+xA)//2), ((yB+yA)//2)), distance)                
                print("Person Center Coord: ", (((xB+xA)//2), ((yB+yA)//2)))
           
                #if no human detected close by, circle around to find one
                if (isMoving == False and distance >=  MAX_DISTANCE):
                  isMoving = True
                  drive.circle_around()
            break

        else: personFound = False

    # Show the resulting frame with detection and distance
  #  cv2.imshow("Detection and Distance", frame)
    if personFound == False:
        foundFailed += 1
        if foundFailed >= 15:
            drive.circle_around()
            foundFailed = 0
        isMoving = True
        print("Im circling\n")
    
    #currently unreachable because of ending for loop condition above
    if isMoving == False:
        drive.stop_motors()
   
    if turn_direction == "forward":
        drive.forward(LOW_SPEED)

    if turn_direction == "left":
        print("Left speed: ", left_speed)
        print("Right speed: ", right_speed)
        drive.left_forward.value = left_speed
        drive.right_forward.value = right_speed
        turn_direction = "none"
    elif turn_direction == "right":
        drive.left_forward.value = left_speed
        drive.right_forward.value = right_speed 
        turn_direction = "none"
    elif turn_direction == "forward":
        drive.forward(LOW_SPEED) 
        turn_direction = "none"
    print("Turn Direction: ", turn_direction)
    print("foundFailed: ", foundFailed) 
    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    print("Person Found: ", personFound)
    print("isMoving: ", isMoving)
cap.release()
cv2.destroyAllWindows()
