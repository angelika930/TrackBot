import cv2
import numpy as np
import tensorflow as tf
import tensorflow_hub as hub

# Load lightweight SSD Mobilenet v2 model (320x320) from TF Hub
detector = hub.load("https://tfhub.dev/tensorflow/ssd_mobilenet_v2/fpnlite_320x320/1")

FOCAL_LENGTH = 446.01
AVERAGE_HEIGHT = 25


def calculate_distance(known_height, focal_length, perceived_height):
    return (known_height * focal_length) / perceived_height


# Start webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    height, width, _ = frame.shape

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
        if scores[i] > 0.5 and classes[i] == 1:  # Class 1 = person in COCO dataset
            y_min, x_min, y_max, x_max = boxes[i]
            xA, yA = int(x_min * width), int(y_min * height)
            xB, yB = int(x_max * width), int(y_max * height)

            # Draw bounding box around detected person
            cv2.rectangle(frame, (xA, yA), (xB, yB), (0, 255, 0), 2)

            # Estimate distance
            perceived_height = xB - xA  # Height of the detected person in pixels
            print("Perceived height: ", perceived_height)
            if perceived_height > 0:
                distance = calculate_distance(AVERAGE_HEIGHT, FOCAL_LENGTH, perceived_height)
                cv2.putText(frame, f"Distance: {distance:.2f} cm", (xA, yA - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Show the resulting frame with detection and distance
    cv2.imshow("Detection and Distance", frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
