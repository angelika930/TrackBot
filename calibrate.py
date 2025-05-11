import cv2
import numpy as np

KNOWN_DISTANCE = 38.1   # cm from camera to object during calibration
KNOWN_WIDTH = 20.32   # cm (width of the object, e.g., credit card)

def find_object_width(image):
    """
    Detect the object and return its width in pixels using contour detection.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(blurred, 50, 200)

    contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    rect = cv2.minAreaRect(largest)
    width = min(rect[1])  # Take the shorter side
    box = cv2.boxPoints(rect)
    box = np.int64(box)
    cv2.drawContours(image, [box], 0, (0, 255, 0), 2)

    return width, image


cap = cv2.VideoCapture(0)

print("Hold the known object at", KNOWN_DISTANCE, "cm and press 'c' to calibrate.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    width_in_pixels, annotated = find_object_width(frame) or (None, frame)

    if width_in_pixels:
        cv2.putText(annotated, f"Width in px: {int(width_in_pixels)}", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

    cv2.imshow("Calibration View", annotated)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('c') and width_in_pixels:
        # === STEP 3: Calculate focal length ===
        focal_length = (width_in_pixels * KNOWN_DISTANCE) / KNOWN_WIDTH
        print(f"✅ Calibrated Focal Length: {focal_length:.2f} pixels")
        break
    elif key == ord('q'):
        print("❌ Calibration canceled.")
        break

cap.release()
cv2.destroyAllWindows()
