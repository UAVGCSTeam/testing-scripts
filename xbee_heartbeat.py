from digi.xbee.devices import DigiMeshDevice, RemoteXBeeDevice
from digi.xbee.models.address import XBee64BitAddress
from digi.xbee.exception import XBeeException
from pymavlink import mavutil
import time
import os

# Use environment variable XBEE_PORT or default to a Windows-style COM port for local testing
PORT = os.environ.get("XBEE_PORT", "COM5")
BAUD = int(os.environ.get("XBEE_BAUD", "9600"))
REMOTE_ADDR = os.environ.get("REMOTE_ADDR", "0013A20041D365C4")

# MAVLink over UDP (baud irrelevant here)
master = mavutil.mavlink_connection('udpout:localhost:14550')

def main():
    xbee = DigiMeshDevice(PORT, BAUD)
    try:
        xbee.open()
        # Sanity: ensure we’re really on DigiMesh+API
        print("Local XBee protocol:", xbee.get_protocol().name)

        remote = RemoteXBeeDevice(xbee, XBee64BitAddress.from_hex_string(REMOTE_ADDR))
        print("XBee connected. Sending MAVLink heartbeats...")
        while True:
            hb = master.mav.heartbeat_encode(
                mavutil.mavlink.MAV_TYPE_GENERIC,
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                0, 0, 0, 0
            )
            payload = hb.pack(master.mav)
            try:
                xbee.send_data(remote, bytes(payload))
                # Debug: show number of bytes and hex
                print(f"Sent MAVLink heartbeat -> {REMOTE_ADDR}: {len(payload)} bytes, hex={payload.hex()}")
            except XBeeException as e:
                # Print precise TX cause if the radio reports one
                status = getattr(e, "status", None)
                if status is not None:
                    print(f"TX Error: {status.name} ({status.value})")
                else:
                    print("TX Error:", repr(e))
            time.sleep(1)

    finally:
        if 'xbee' in locals() and xbee.is_open():
            xbee.close()
            print("XBee connection closed.")

if __name__ == "__main__":
    main()