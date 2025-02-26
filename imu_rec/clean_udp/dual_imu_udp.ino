import socket
import json
import threading

# ==================== UDP Settings ====================
UDP_IP = "0.0.0.0"  # Listen on all network interfaces
UDP_PORT_IMU1 = 8888  # IMU1 Data Port
UDP_PORT_IMU2 = 8889  # IMU2 Data Port

# ==================== Function to Receive Data ====================
def receive_udp_data(port, imu_name):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP Socket
    sock.bind((UDP_IP, port))
    print(f"Listening for {imu_name} data on port {port}...")

    while True:
        try:
            data, addr = sock.recvfrom(1024)  # Receive UDP packet (max 1024 bytes)
            decoded_data = data.decode('utf-8')  # Decode to string
            json_data = json.loads(decoded_data)  # Parse JSON

            pitch = json_data.get("pitch", "N/A")
            roll = json_data.get("roll", "N/A")

            print(f"{imu_name} | Pitch: {pitch:.2f}, Roll: {roll:.2f} | From: {addr}")
        except json.JSONDecodeError:
            print(f"Error decoding JSON from {imu_name}: {data}")
        except Exception as e:
            print(f"Error in {imu_name} receiver: {e}")

# ==================== Start UDP Listeners ====================
thread_imu1 = threading.Thread(target=receive_udp_data, args=(UDP_PORT_IMU1, "IMU1"))
thread_imu2 = threading.Thread(target=receive_udp_data, args=(UDP_PORT_IMU2, "IMU2"))

thread_imu1.start()
thread_imu2.start()

thread_imu1.join()
thread_imu2.join()
