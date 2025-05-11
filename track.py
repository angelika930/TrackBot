import cv2
import numpy as np

#Known constants
#KNOWN_WIDTH = 20.32  # cm (real width of the object)
FOCAL_LENGTH = 446.01  # calibrated focal length from calibrate.py
AVERAGE_HEIGHT = 152.4
def find_object_width(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #convert to grayscale
    #blurred = cv2.GaussianBlur(gray, (5, 5), 0) #average out noise in image with 5x5 matrix
    #edged = cv2.Canny(blurred, 50, 200)

    contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #find boundaries of image
    if not contours: # should return numpy array of endpoints of contours
        return None, image

    largest = max(contours, key=cv2.contourArea)
    rect = cv2.minAreaRect(largest)

    width = min(rect[1])  # the shorter side of the bounding box
    box = cv2.boxPoints(rect)
    box = np.intp(box)
    cv2.drawContours(image, [box], 0, (0, 255, 0), 2)
    print(width)
    print("Image: ")
    print(image)
    return width, image

def calculate_distance(known_width, focal_length, perceived_width):
    return (known_width * focal_length) / perceived_width

# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# Start usb webcam
cap = cv2.VideoCapture(1)

# the output will be written to output.avi
out = cv2.VideoWriter(
     'output.avi',
     cv2.VideoWriter_fourcc(*'MJPG'),
     15.,
     (640,480))

count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        break

    #resizing for faster detection
    frame = cv2.resize(frame, (640, 480))
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    #find object width in pixels
    # width_in_pixels, annotated = find_object_width(frame)

    boxes, weights = hog.detectMultiScale(frame, winStride=(4, 4), scale=1.1)

    boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])
    for i, (xA, yA, xB, yB) in enumerate(boxes):
        # display the detected boxes in the colour picture
        if weights[i] >= 0.5:
            cv2.rectangle(frame, (xA, yA), (xB, yB),
                          (0, 255, 0), 2)
            #print("about to display")

            #cv2.imshow("Live Distance Estimation", frame)
            perceived_height = yB - yAv

            # Calculate the distance to the person using height
            if perceived_height > 0:  # Avoid division by zero
                distance = calculate_distance(AVERAGE_HEIGHT, FOCAL_LENGTH, perceived_height)
                cv2.putText(frame, f"Distance: {distance:.2f} cm", (xA, yA - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0), 2)

    # Write the output video
    out.write(frame.astype('uint8'))
    # Display the resulting frame

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

out.release()
cap.release()
cv2.destroyAllWindows()
