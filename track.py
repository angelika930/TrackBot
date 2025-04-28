import cv2
import numpy as np

#Known constants
KNOWN_WIDTH = 40.64  # cm (real width of the object)
FOCAL_LENGTH = 566.16  # calibrated focal length from calibrate.py

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

    return width, image

def calculate_distance(known_width, focal_length, perceived_width):
    return (known_width * focal_length) / perceived_width

# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# Start usb webcam
cap = cv2.VideoCapture(0)

# the output will be written to output.avi
out = cv2.VideoWriter(
    'output.avi',
    cv2.VideoWriter_fourcc(*'MJPG'),
    15.,
    (640,480))


while True:
    ret, frame = cap.read()
    if not ret:
        break

    #resizing for faster detection
    frame = cv2.resize(frame, (640, 480))
    #find object width in pixels
    width_in_pixels, annotated = find_object_width(frame)
    # detect people in the image
    # returns the bounding boxes for the detected objects
    boxes, weights = hog.detectMultiScale(annotated, winStride=(8, 8))

    boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])
    for (xA, yA, xB, yB) in boxes:
        # display the detected boxes in the colour picture
        cv2.rectangle(annotated, (xA, yA), (xB, yB),
                      (0, 255, 0), 2)


    # calculate pixel width of the person
        person_width_px = xB - xA

    # Display the resulting frame
    #cv2.imshow('frame', frame)
        if person_width_px > 0:
            distance_cm = calculate_distance(40, FOCAL_LENGTH, person_width_px)
            cv2.putText(annotated, f"{distance_cm:.2f} cm", (xA, yA - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("Live Distance Estimation", annotated)
        # Write the output video
        out.write(annotated.astype('uint8'))

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
