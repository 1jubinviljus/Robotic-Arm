import cv2
import mediapipe as mp

# Initialize Mediapipe Hands module and drawing utilities.
mp_hands = mp.solutions.hands  # sets up landmark detector
mp_drawing = mp.solutions.drawing_utils  # helps draw the landmarks

# Open the default webcam. Index 0 for my built-in webcam
cap = cv2.VideoCapture(0)

# Create a Hands object
with mp_hands.Hands(
    # Basic parameters to tune detection and tracking
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7) as hands:

    # Start processing frames until webcam is closed or user stops.
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty frame.")  # Skip failed frames
            continue

        # Flip image horizontally for a mirrored image
        image = cv2.flip(image, 1)

        # Converts color space so Mediapipe(RGB) can interpret OpenCV(BGR)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Improve performance by marking image as not writable
        image_rgb.flags.writeable = False

        # Run hand landmark detection on the RGB frame
        results = hands.process(image_rgb)

        # Set writable back to True so we can draw on the image
        image_rgb.flags.writeable = True

        # Convert back from RGB to BGR for OpenCV display.
        image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

        # If hands are detected in the frame...
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw all hand landmarks and connections on the image
                mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # Landmark 8 corresponds to the tip of the index finger
                fingertip = hand_landmarks.landmark[8]

                # Convert normalized landmark coordinates (0 to 1) to pixel positions
                h, w, _ = image.shape
                cx, cy = int(fingertip.x * w), int(fingertip.y * h)

                # Draw a green circle on the fingertip
                cv2.circle(image, (cx, cy), 3, (0, 255, 0), cv2.FILLED)

                print(f"Fingertip coordinates: {(cx, cy)}")

        # Show the image with drawings in a window titled 'Fingertip Tracking'
        cv2.imshow('Fingertip Tracking', image)

        # Wait for 5ms and check if the user pressed the 'Esc' key (ASCII 27) to exit
        if cv2.waitKey(5) & 0xFF == 27:
            break

# Release the webcam and close all OpenCV windows when done.
cap.release()
cv2.destroyAllWindows()
