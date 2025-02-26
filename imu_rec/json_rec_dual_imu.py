import requests
import socket
import json
import time

# Change these to match your ESP8266's details
HTTP_SERVER_IP = "192.168.17.121"  # ESP8266 IP
HTTP_SERVER_PORT = 80
UDP_PORT1 = 8888  # Port for Sensor 1 data
UDP_PORT2 = 8889  # Port for Sensor 2 data

# Function to fetch data from HTTP
def fetch_http_data():
    try:
        response = requests.get(f"http://{HTTP_SERVER_IP}:{HTTP_SERVER_PORT}/data")
        if response.status_code == 200:
            data = response.json()
            print("\n[HTTP] Sensor 1 - Pitch: {:.2f}, Roll: {:.2f}, Yaw: {:.2f}".format(
                data['sensor1']['pitch'], data['sensor1']['roll'], data['sensor1']['yaw']
            ))
            print("[HTTP] Sensor 2 - Pitch: {:.2f}, Roll: {:.2f}, Yaw: {:.2f}".format(
                data['sensor2']['pitch'], data['sensor2']['roll'], data['sensor2']['yaw']
            ))
        else:
            print(f"[HTTP] Failed to fetch data: {response.status_code}")
    except Exception as e:
        print(f"[HTTP] Error: {e}")

# Function to receive UDP data
def receive_udp_data():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", UDP_PORT1))  # Bind to all available network interfaces for UDP_PORT1
    sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock2.bind(("", UDP_PORT2))  # Bind to UDP_PORT2

    print(f"[UDP] Listening on ports {UDP_PORT1} and {UDP_PORT2}...")

    while True:
        try:
            # UDP Sensor 1
            data1, addr1 = sock.recvfrom(1024)
            sensor1_data = json.loads(data1.decode())
            print("\n[UDP] Sensor 1 - Pitch: {:.2f}, Roll: {:.2f}, Yaw: {:.2f}".format(
                sensor1_data['pitch'], sensor1_data['roll'], sensor1_data['yaw']
            ))

            # UDP Sensor 2
            data2, addr2 = sock2.recvfrom(1024)
            sensor2_data = json.loads(data2.decode())
            print("[UDP] Sensor 2 - Pitch: {:.2f}, Roll: {:.2f}, Yaw: {:.2f}".format(
                sensor2_data['pitch'], sensor2_data['roll'], sensor2_data['yaw']
            ))

        except Exception as e:
            print(f"[UDP] Error: {e}")
            break

# Run both HTTP and UDP fetching in a loop
if __name__ == "__main__":
    from threading import Thread

    # Run UDP receiver in a separate thread
    udp_thread = Thread(target=receive_udp_data, daemon=True)
    udp_thread.start()

    # Fetch HTTP data periodically
    while True:
        fetch_http_data()
        time.sleep(1)
