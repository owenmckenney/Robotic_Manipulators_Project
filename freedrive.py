import socket
import time

# Define UR5 IP and port
ip = "10.168.18.103"  # Replace with your robot's IP address
port = 30002           # Port for URScript commands

# Establish socket connection
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    sock.connect((ip, port))
    print("Connected to UR5.")
except Exception as e:
    print(f"Connection error: {e}")
    exit(1)

# URScript command to enable freedrive mode, log positions, and then exit freedrive mode
urscript_command = '''
def freedrive_and_log():    
    while True:
        freedrive_mode()
    end
end
freedrive_and_log()
'''

# Send the URScript command
try:
    sock.sendall(urscript_command.encode('utf-8'))
    print("Freedrive and logging script sent to UR5.")
except Exception as e:
    print(f"Error sending URScript command: {e}")


# Close the socket connection
sock.close()
print("Socket closed.")
