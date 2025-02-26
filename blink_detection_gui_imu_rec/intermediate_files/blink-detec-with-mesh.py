import sys
import cv2
import mediapipe as mp
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QLabel, QPushButton, QWidget, QStackedWidget
)
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from collections import deque
import time


class EyeBlinkDetector(QThread):
    blink_detected = pyqtSignal(int)  # Emit the number of blinks detected (1 or 2)

    def __init__(self, camera_url):
        super().__init__()
        self.camera_url = camera_url
        self.running = True
        self.blink_threshold = 0.5  # EAR threshold for blink detection
        self.double_blink_max_time = 400  # Max time (ms) for double blink
        self.last_blink_time = None
        self.single_blink_emitted = False
        self.ear_history = deque(maxlen=5)

        # Initialize MediaPipe FaceMesh
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.drawing_spec = self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=1, circle_radius=1) 

    def run(self):
        cap = cv2.VideoCapture(self.camera_url)
        if not cap.isOpened():
            print("Error: Could not access the camera.")
            return

        print("Camera accessed successfully.")

        while self.running:
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read a frame.")
                break

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.face_mesh.process(rgb_frame)

            if results.multi_face_landmarks:
                for face_landmarks in results.multi_face_landmarks:
                    self.mp_drawing.draw_landmarks(
                    frame,
                    face_landmarks,
                    self.mp_face_mesh.FACEMESH_TESSELATION,  
                    landmark_drawing_spec=self.drawing_spec,
                    connection_drawing_spec=self.drawing_spec,
                )

                    left_eye_ratio = self.calculate_eye_ratio(
                        face_landmarks.landmark, [33, 160, 159, 158, 153, 144, 145, 133]
                    )
                    right_eye_ratio = self.calculate_eye_ratio(
                        face_landmarks.landmark, [362, 385, 387, 386, 374, 380, 381, 263]
                    )

                    smoothed_ear = (left_eye_ratio + right_eye_ratio) / 2
                    self.ear_history.append(smoothed_ear)
                    self.detect_blink(sum(self.ear_history) / len(self.ear_history))

            cv2.imshow("Camera Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    def detect_blink(self, smoothed_ear):
        current_time = time.time() * 1000  # Current time in milliseconds

        if smoothed_ear < self.blink_threshold:  # Eye closed
            if self.last_blink_time is None:
                self.last_blink_time = current_time
                self.single_blink_emitted = False

        elif smoothed_ear >= self.blink_threshold:  # Eye open
            if self.last_blink_time is not None:
                elapsed_time = current_time - self.last_blink_time
                if elapsed_time < self.double_blink_max_time:
                    if not self.single_blink_emitted:
                        self.blink_detected.emit(1)  # Single blink detected
                        print("Single Blink Detected!")
                        self.single_blink_emitted = True
                else:
                    self.blink_detected.emit(2)  # Double blink detected
                    print("Double Blink Detected!")
                self.last_blink_time = None

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

        # Main Options
        self.main_options = QWidget()
        self.main_layout = QVBoxLayout()
        self.main_options.setLayout(self.main_layout)

        self.sideways_button = QPushButton("Sideways")
        self.main_layout.addWidget(self.sideways_button)

        self.upper_body_button = QPushButton("Upper Body")
        self.main_layout.addWidget(self.upper_body_button)

        self.lower_body_button = QPushButton("Lower Body")
        self.main_layout.addWidget(self.lower_body_button)

        self.options_stack.addWidget(self.main_options)

        # Sub-options
        self.sub_options_widgets = {
            "Sideways": self.create_sub_options(["Right", "Left", "Back"]),
            "Upper Body": self.create_sub_options(["30°", "45°", "60°", "Back"]),
            "Lower Body": self.create_sub_options(["30°", "45°", "60°", "Back"]),
        }

        self.option_list = [self.sideways_button, self.upper_body_button, self.lower_body_button]
        self.current_index = 0
        self.highlight_option(self.main_options)  # Start with the first option highlighted

        # Initialize sub-option highlighted states
        self.sub_option_highlighted = {
            "Sideways": 0,
            "Upper Body": 0,
            "Lower Body": 0
        }

        self.eye_blink_detector = EyeBlinkDetector(camera_url)
        self.eye_blink_detector.blink_detected.connect(self.handle_blink)
        self.eye_blink_detector.start()

    def create_sub_options(self, options):
        widget = QWidget()
        layout = QVBoxLayout()
        widget.setLayout(layout)
        self.options_stack.addWidget(widget)

        for i, option in enumerate(options):
            button = QPushButton(option)
            layout.addWidget(button)

            # Highlight the first option
            if i == 0:
                button.setStyleSheet("background-color: yellow;")
        return widget

    def handle_blink(self, blink_count):
        if blink_count == 1:
            current_widget = self.options_stack.currentWidget()
            if isinstance(current_widget, QWidget):
                self.highlight_option(current_widget)
        elif blink_count == 2:
            self.select_option()

    def highlight_option(self, current_widget):
        buttons = current_widget.findChildren(QPushButton)
        for i, button in enumerate(buttons):
            if self.current_index == i:
                button.setStyleSheet("background-color: yellow;")
            else:
                button.setStyleSheet("")

        self.current_index = (self.current_index + 1) % len(buttons)

    def select_option(self):
        current_widget = self.options_stack.currentWidget()
        if isinstance(current_widget, QWidget):
            buttons = current_widget.findChildren(QPushButton)
            selected_button = buttons[self.current_index - 1]  # Adjust for cycling
            print(f"Selected: {selected_button.text()}")

            if selected_button.text() == "Back":
                self.options_stack.setCurrentWidget(self.main_options)
                self.current_index = 0  # Reset index
                self.highlight_option(self.main_options)
            elif selected_button.text() in self.sub_options_widgets:
                self.options_stack.setCurrentWidget(self.sub_options_widgets[selected_button.text()])
                self.current_index = 0
                self.highlight_option(self.sub_options_widgets[selected_button.text()])

    def closeEvent(self, event):
        self.eye_blink_detector.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    camera_url = 0  # Default camera
    gui = EyeControlGUI(camera_url)
    gui.show()
    sys.exit(app.exec_())
