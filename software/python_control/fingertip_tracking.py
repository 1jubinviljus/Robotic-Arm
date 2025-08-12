import cv2
import mediapipe as mp
import math

# Initialize Mediapipe Hands module and drawing utilities.
mp_hands = mp.solutions.hands  # sets up landmark detector
mp_drawing = mp.solutions.drawing_utils  # helps draw the landmarks

# Open the default webcam. Index 0 for built-in webcam
cap = cv2.VideoCapture(0)

# Camera and workspace parameters (adjust these to your setup)
W = 1280  # expected camera frame width in pixels
H = 720   # expected camera frame height in pixels
FOVx = 65  # horizontal field of view in degrees (approximate)
D = 42    # distance from camera to robot workspace plane in cm 
          #(robot sjoulder joint in the bottom left,adjust webcam down until the top of the robot is at the top of the frame)

# Precompute scaling factors for pixel -> cm conversion
fovx_rad = math.radians(FOVx)
view_width = 2 * D * math.tan(fovx_rad / 2)  # real-world width at distance D in cm
scale_x = view_width / W

aspect_ratio = H / W
FOVy = FOVx * aspect_ratio
fovy_rad = math.radians(FOVy)
view_height = 2 * D * math.tan(fovy_rad / 2)  # real-world height at distance D in cm
scale_y = view_height / H

with mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7) as hands:

    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty frame.")  # Skip failed frames
            continue

        image = cv2.flip(image, 1)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        results = hands.process(image_rgb)
        image_rgb.flags.writeable = True
        image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                fingertip = hand_landmarks.landmark[8]
                h, w, _ = image.shape
                cx, cy = int(fingertip.x * w), int(fingertip.y * h)

                # Draw green circle on fingertip
                cv2.circle(image, (cx, cy), 10, (0, 255, 0), cv2.FILLED)

                # Map pixel coords to real-world coords with bottom-left as (0,0)
                X = cx * scale_x
                Y = (H - cy) * scale_y  # flip y-axis so bottom of image is 0

                print(f"Fingertip pixel coords: ({cx}, {cy})")
                print(f"Mapped real-world coords: X={X:.2f} cm, Y={Y:.2f} cm")

        cv2.imshow('Fingertip Tracking', image)

        if cv2.waitKey(5) & 0xFF == 27:
            break

cap.release()
cv2.destroyAllWindows()
