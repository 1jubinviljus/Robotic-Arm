import cv2
import mediapipe as mp
import math
import threading

class FingertipTracker:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Webcam not accessible!")

        self.W = 1280
        self.H = 720
        self.FOVx = 65
        self.D_cm = 42
        self.D = self.D_cm / 100.0
        fovx_rad = math.radians(self.FOVx)
        view_width = 2 * self.D * math.tan(fovx_rad / 2)
        self.scale_x = view_width / self.W
        aspect_ratio = self.H / self.W
        FOVy = self.FOVx * aspect_ratio
        fovy_rad = math.radians(FOVy)
        view_height = 2 * self.D * math.tan(fovy_rad / 2)
        self.scale_y = view_height / self.H

        self.fingertip_pos = None
        self.running = False
        self.thread = None
        self.lock = threading.Lock()

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self):
        mp_drawing = mp.solutions.drawing_utils
        while self.running and self.cap.isOpened():
            success, image = self.cap.read()
            if not success:
                continue
            image = cv2.flip(image, 1)
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image_rgb.flags.writeable = False
            results = self.hands.process(image_rgb)
            image_rgb.flags.writeable = True
            image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]
                fingertip = hand_landmarks.landmark[8]
                h, w, _ = image.shape
                cx, cy = int(fingertip.x * w), int(fingertip.y * h)

                X = cx * self.scale_x
                Y = (self.H - cy) * self.scale_y

                with self.lock:
                    self.fingertip_pos = (X, Y)

                cv2.circle(image, (cx, cy), 10, (0, 255, 0), cv2.FILLED)
                mp_drawing.draw_landmarks(image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

            cv2.imshow('Fingertip Tracking', image)
            if cv2.waitKey(5) & 0xFF == 27:
                self.stop()
                break

    def get_fingertip_position(self):
        with self.lock:
            return self.fingertip_pos

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        self.cap.release()
        cv2.destroyAllWindows()
