import socket

# First ESP32's IP address and port
esp32_ip1 = "192.168.103.164"  # Replace with the actual IP address of the first ESP32
esp32_ip2 = "192.168.103.123"
esp32_port = 1234

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_command(command):
    """
    Send a command to the ESP32 over the persistent TCP connection.
    """
    # Send data to the first ESP32
    sock.sendto(command.encode(), (esp32_ip1, esp32_port))
    sock.sendto(command.encode(), (esp32_ip2, esp32_port))
