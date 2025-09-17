from digi.xbee.devices import DigiMeshDevice
from pymavlink import mavutil
import time
import os

# --- XBee Configuration ---
LOCAL_XBEE_PORT = os.environ.get("XBEE_PORT", "COM5")
BAUD = int(os.environ.get("XBEE_BAUD", "9600"))

def hexdump(b: bytes) -> str:
    return ' '.join(f"{x:02x}" for x in b)

try:
    # Initialize local XBee device
    local_xbee = DigiMeshDevice(LOCAL_XBEE_PORT, BAUD)
    local_xbee.open()

    print(f"XBee connected on {LOCAL_XBEE_PORT} @ {BAUD}. Protocol: {local_xbee.get_protocol().name}")

    while True:
        # Read data from XBee (blocking read with timeout is possible via read_data(1))
        xbee_message = local_xbee.read_data()

        if xbee_message is not None:
            mavlink_data = xbee_message.data
            print(f"RAW {len(mavlink_data)} bytes: {hexdump(mavlink_data)}")
            # Try to parse MAVLink using pac bytes; robust parsing would accumulate a stream.
            try:
                # Attempt to decode a full MAVLink message (may raise on partial frames)
                msg = mavutil.mavlink.MAVLink_message.decode(mavlink_data)
                print(f"Received MAVLink message: {msg.get_type()}")
            except Exception as e:
                print(f"Decode error (may be partial frame): {e}")

        time.sleep(0.1)  # Check for new data frequently

except Exception as e:
    print(f"Error: {e}")

finally:
    if 'local_xbee' in locals() and local_xbee.is_open():
        local_xbee.close()
        print("XBee connection closed.")