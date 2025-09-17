
from digi.xbee.devices import DigiMeshDevice, RemoteXBeeDevice
from digi.xbee.models.address import XBee64BitAddress
from digi.xbee.exception import XBeeException
from pymavlink import mavutil
import time

PORT = "/dev/cu.usbserial-AQ015EBI"   # from your diagnostic
BAUD = 9600                           # BD=3 on the radio
REMOTE_ADDR = "0013A20041D365C4"      # remote node 64-bit (SH+SL)

# MAVLink over UDP (baud irrelevant here)
master = mavutil.mavlink_connection('udpout:localhost:14550')

def main():
    xbee = DigiMeshDevice(PORT, BAUD)
    try:
        xbee.open()
        # Sanity: ensure we’re really on DigiMesh+API
        assert xbee.get_protocol().name == "DIGI_MES"
        "H"

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
                print("Sent MAVLink heartbeat")
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
