import cv2
import numpy as np

# Load image
image = cv2.imread('contrail.jpg')

# Preprocessing
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

# Edge Detection
edges = cv2.Canny(blurred_image, 50, 150)

# Line Detection (Hough Transform)
lines = cv2.HoughLines(edges, 1, np.pi / 180, 150)

# Postprocessing and Display
for line in lines:
    rho, theta = line[0]
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * (a))
    x2 = int(x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * (a))
    
    # Filter lines based on angle and length to find likely contrails
    if 1.55 < np.abs(theta) < 1.58:  # Close to horizontal lines
        cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)

cv2.imshow('Detected Contrails', image)
cv2.waitKey(0)
cv2.destroyAllWindows()