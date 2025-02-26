import sys
import cv2
import mediapipe as mp
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QPushButton, QWidget, QStackedWidget
from PyQt5.QtCore import QThread, pyqtSignal, QTimer


class EyeBlinkDetector(QThread):
    blink_detected = pyqtSignal(int)  # Emit number of blinks detected (1 or 2)

    def __init__(self, camera_url):
        super().__init__()
        self.camera_url = camera_url
        self.running = True
        self.last_blink_time = None
        self.blink_threshold = 0.5  # Adjust this for sensitivity
        self.double_blink_time = 500  # Maximum time (ms) for a double blink

        # Initialize MediaPipe FaceMesh
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

    def run(self):
        cap = cv2.VideoCapture(self.camera_url)
        if not cap.isOpened():
            print("Error: Could not access the camera.")
            return

        print("Camera accessed successfully.")
        mp_drawing = mp.solutions.drawing_utils
        mp_drawing_styles = mp.solutions.drawing_styles

        while self.running:
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read a frame.")
                break

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.face_mesh.process(rgb_frame)

            if results.multi_face_landmarks:
                for face_landmarks in results.multi_face_landmarks:
                    mp_drawing.draw_landmarks(
                        frame,
                        face_landmarks,
                        mp.solutions.face_mesh.FACEMESH_TESSELATION,
                        landmark_drawing_spec=None,
                        connection_drawing_spec=mp_drawing_styles.get_default_face_mesh_tesselation_style(),
                    )

                    left_eye_ratio = self.calculate_eye_ratio(
                        face_landmarks.landmark, [33, 160, 159, 158, 153, 144, 145, 133]
                    )
                    right_eye_ratio = self.calculate_eye_ratio(
                        face_landmarks.landmark, [362, 385, 387, 386, 374, 380, 381, 263]
                    )

                    if left_eye_ratio < self.blink_threshold and right_eye_ratio < self.blink_threshold:
                        self.detect_blink()

            cv2.imshow("Camera Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    def detect_blink(self):
        import time
        current_time = time.time() * 1000  # Current time in milliseconds
        if self.last_blink_time and (current_time - self.last_blink_time) < self.double_blink_time:
            self.blink_detected.emit(2)  # Double blink detected
            self.last_blink_time = None
        else:
            self.blink_detected.emit(1)  # Single blink detected
            self.last_blink_time = current_time

    def stop(self):
        self.running = False
        self.wait()

    @staticmethod
    def calculate_eye_ratio(landmarks, indices):
        horizontal_distance = (
            (landmarks[indices[0]].x - landmarks[indices[4]].x) ** 2
            + (landmarks[indices[0]].y - landmarks[indices[4]].y) ** 2
        ) ** 0.5
        vertical_distance_1 = (
            (landmarks[indices[1]].x - landmarks[indices[5]].x) ** 2
            + (landmarks[indices[1]].y - landmarks[indices[5]].y) ** 2
        ) ** 0.5
        vertical_distance_2 = (
            (landmarks[indices[2]].x - landmarks[indices[6]].x) ** 2
            + (landmarks[indices[2]].y - landmarks[indices[6]].y) ** 2
        ) ** 0.5

        eye_ratio = (vertical_distance_1 + vertical_distance_2) / (2.0 * horizontal_distance)
        return eye_ratio


class EyeControlGUI(QMainWindow):
    def __init__(self, camera_url):
        super().__init__()

        self.setWindowTitle("Eye Control GUI")
        self.setGeometry(100, 100, 400, 300)

        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)

        self.layout = QVBoxLayout()
        self.main_widget.setLayout(self.layout)

        self.label = QLabel("Welcome! Blink to navigate.")
        self.layout.addWidget(self.label)

        self.options_stack = QStackedWidget()
        self.layout.addWidget(self.options_stack)

        self.main_options = QWidget()
        self.main_layout = QVBoxLayout()
        self.main_options.setLayout(self.main_layout)
        self.options_stack.addWidget(self.main_options)

        self.sideways_button = QPushButton("Sideways")
        self.main_layout.addWidget(self.sideways_button)

        self.lower_body_button = QPushButton("Lower Body")
        self.main_layout.addWidget(self.lower_body_button)

        self.upper_body_button = QPushButton("Upper Body")
        self.main_layout.addWidget(self.upper_body_button)

        self.option_list = [self.sideways_button, self.lower_body_button, self.upper_body_button]
        self.current_index = 0
        self.highlight_option()  # Start with the first option highlighted

        self.eye_blink_detector = EyeBlinkDetector(camera_url)
        self.eye_blink_detector.blink_detected.connect(self.handle_blink)
        self.eye_blink_detector.start()

    def handle_blink(self, blink_count):
        if blink_count == 1:
            self.highlight_option()
        elif blink_count == 2:
            self.select_option()

    def highlight_option(self):
        for i, button in enumerate(self.option_list):
            if i == self.current_index:
                button.setStyleSheet("background-color: yellow;")
            else:
                button.setStyleSheet("")

        self.current_index = (self.current_index + 1) % len(self.option_list)

    def select_option(self):
        selected_button = self.option_list[self.current_index - 1]
        selected_button.click()

    def closeEvent(self, event):
        self.eye_blink_detector.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    camera_url = 0  # Default camera
    gui = EyeControlGUI(camera_url)
    gui.show()
    sys.exit(app.exec_())
