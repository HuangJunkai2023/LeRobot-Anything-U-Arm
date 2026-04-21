import serial
import time

SERIAL_PORT = '/dev/ttyUSB0'  # Replace with your serial port
BAUD_RATE = 115200
CURRENT_ID = 0
NEW_ID = 7

with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5) as ser:
    # Modify ID: change original ID to the target ID.
    cmd = f'#{CURRENT_ID:03d}PID{NEW_ID:03d}!\r\n'.encode('ascii')
    ser.write(cmd)
    print(f"Sent ID modification command: {cmd.decode().strip()}")
    time.sleep(0.5)

    if ser.in_waiting:
        response = ser.read(ser.in_waiting).decode(errors='ignore')
        print(f"Response: {response}")
    else:
        print("No servo response received (modification may have succeeded but no feedback)")
