import cv2

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Camera not accessible.")
else:
    print("Camera accessed successfully.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read a frame.")
        break
    cv2.imshow("Test Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()