# basic script to see if the xbees can just communicate!!



# import serial
from xbee import XBee
import time

# Replace with your serial port and baud rate
SERIAL_PORT = '/dev/ttyUSB1'  # Example: 'COM4' for Windows
BAUD_RATE = 9600

# Open the serial port
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

# Create an XBee object
xbee = XBee(ser)

def data_received_callback(data):
    """Callback function for received data."""
    # The 'rf_data' field contains the received message
    if 'rf_data' in data:
        message = data['rf_data'].decode('utf-8')
        print(f"Received: {message}")

try:
    print("Receiver script running. Waiting for data...")
    while True:
        # Process incoming data. This is a non-blocking call.
        xbee.at(command='nd', frame_id='A')  # A simple command to keep the loop active
        xbee.read_frame()  # This will trigger the callback if data is available
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting...")

finally:
    # Always close the serial port
    ser.close()
