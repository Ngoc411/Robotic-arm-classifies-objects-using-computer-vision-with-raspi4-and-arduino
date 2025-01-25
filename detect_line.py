import cv2
import numpy as np
import math

def detect_line(frame):
    """
    Function to detect the longest line in a given frame using Canny edge detection
    and Hough Line Transformation.
    """
    max_len_line = 0
    max_len_line_index = None

    # Convert to grayscale image
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Use Canny to find edges
    edges = cv2.Canny(gray, 50, 200, None, 3)

    # Detect lines using Probabilistic Hough Transform
    linesP = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, None, 60, 60)

    if linesP is not None:
        for i, lineP in enumerate(linesP):
            # Calculate the length of each detected line
            len_line = math.sqrt((lineP[0][2] - lineP[0][0])**2 + (lineP[0][3] - lineP[0][1])**2)

            if len_line > max_len_line:
                max_len_line = len_line
                max_len_line_index = i

            # Draw each detected line on the frame
            cv2.line(frame, (lineP[0][0], lineP[0][1]), (lineP[0][2], lineP[0][3]), (0, 0, 255), 3, cv2.LINE_AA)

        # If a longest line is found, calculate its angle
        if max_len_line != 0 and max_len_line_index is not None:
            degree_theta = math.degrees(
                math.atan2(
                    linesP[max_len_line_index][0][3] - linesP[max_len_line_index][0][1],
                    linesP[max_len_line_index][0][2] - linesP[max_len_line_index][0][0]
                )
            )
            degree_theta = -degree_theta
            if degree_theta < 0:
                degree_theta = 180 + degree_theta
            if 178 <= degree_theta <= 180 or 0 <= degree_theta <= 2:
                degree_theta = 0

            print(f"Longest Line Angle: {degree_theta}°")

# Example usage of the detect_line function
if __name__ == "__main__":
    cap = cv2.VideoCapture(1)  # Open the default camera
    cap.set(3, 640)  # Set frame width
    cap.set(4, 480)  # Set frame height

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        detect_line(frame)  # Detect and annotate lines in the frame

        # Show the frame with detected lines
        cv2.imshow('Line Detection', frame)

        # Exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
