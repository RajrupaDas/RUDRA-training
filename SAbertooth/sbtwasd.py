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
(vr) root@rajrupa-das-Dell-G15-5535:/home/rajrupa-das/vr# cat sbtwasd.py
import serial
import time
import keyboard

# Attempt to create a serial connection
try:
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Adjust as needed
    print(f"Connected to {ser.portstr}")  # Confirm connection
except serial.SerialException as e:
    print(f"Error: {e}")
    exit(1)

def send_command(command):
    try:
        # Ensure command is sent as a single byte
        command_byte = bytes([command])
        ser.write(command_byte)
        print(f"Command sent: { command} (0x{command:02X})")  # Show command in hex for clarity
    except Exception as e:
        print(f"command failed")

while True:
    if keyboard.is_pressed('w'):
        send_command(64+30) 
        send_command(192+30)
        time.sleep(0.1) 
        
    elif keyboard.is_pressed('s'): 
        send_command(64-30) 
        send_command(192-30)
        time.sleep(0.1)
        
    elif keyboard.is_pressed('a'):
        send_command(64-30) 
        send_command(192+30)
        time.sleep(0.1)
        
    elif keyboard.is_pressed('d'):
        send_command(64+30) 
        send_command(192-30) 
        time.sleep(0.1)

    elif keyboard.is_pressed('t'):
        send_command(64)
        send_command(192)
        time.sleep(0.1)

    elif keyboard.is_pressed('q'):
       time.sleep(0.1)
       send_command(64)  
       send_command(192)  

    elif keyboard.is_pressed('c'):
       time.sleep(0.1)
       send_command(64)
       send_command(192)
       ser.close()
       break
