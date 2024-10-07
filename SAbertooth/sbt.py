import serial
import time

try:
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  
    print(f"Connected") 
except serial.SerialException as e:
    print(f"Error: {e}")
    exit(1)

def send_command(command):
    try:
        # Ensure command is sent as a single byte
        command_byte = bytes([command])
        ser.write(command_byte)
        print(f"Command sent: {command}") 
    except Exception as e:
        print(f"Failed to send command: {e}")

# Move motor forward
send_command(64+30)  # Move motor 1 forward at medium speed

time.sleep(5)  # Simulate running for 2 seconds

# Stop motors
send_command(64)  # Stop motor 1

time.sleep(1)  # Simulate pause for a second

# Move motor backward
send_command(64-30)  # Move motor 1 backward at medium speed

time.sleep(2)

# Stop motors
send_command(64)  # Stop motor 1

ser.close()
