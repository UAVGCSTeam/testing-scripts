from pymavlink import mavutil
import time
from serial import SerialException
import sys
import glob 

# MAC_IP = "0.0.0.0" 
MAC_IP = "127.0.0.1" 
# MAC_IP = "192.168.1.237" 
UDP_PORT = "14550"
TIMEOUT_S = 5 # seconds 
SERIAL = True
BAUD = "57600"
usb_ports = ''

try: 
    # == CONNECT TO THE AUTOPILOT == 
    print(f"[{time.strftime("%I:%M:%S %p")}] … Attempting to make object and connection")
    if SERIAL: 
        print("Using serial connection")
        usb_ports = glob.glob('/dev/cu.usb*')
        if not usb_ports:
            raise Exception("No USB serial ports found matching /dev/cu.usb*")
        port = usb_ports[0]  # Use the first match
        if len(usb_ports) > 1:
            print(f"Multiple USB ports found: {usb_ports}. Using {port}")

        mav = mavutil.mavlink_connection(port, baud=BAUD)
        print(f"[{time.strftime("%I:%M:%S %p")}] ✔︎ Connected on {port} @ baud {BAUD}")
    else: 
        print("Using UDP connection")
        mav = mavutil.mavlink_connection(f'udp:{MAC_IP}:{UDP_PORT}', source_system=0)
        print(f"[{time.strftime("%I:%M:%S %p")}] ✔︎ Connected on {MAC_IP}:{UDP_PORT}")


    print("Listening for telemetry...")

    try:
        while True:
            # Listen for any new message from the MAVLink stream
            msg = mav.recv_match(blocking=True)
            
            if msg is not None:
                # Print message info if it’s a known message type
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    print(f"Timestamp: {time.time()}")
                    print(f"Latitude: {msg.lat / 1e7}°")
                    print(f"Longitude: {msg.lon / 1e7}°")
                    print(f"Altitude: {msg.relative_alt / 1000.0} m (AGL)")
                    print(f"Velocity: {msg.vx}, {msg.vy}, {msg.vz} (m/s)\n")

                elif msg.get_type() == 'GPS_RAW_INT':
                    print(f"Timestamp: {time.time()}")
                    print(f"GPS Fix Type: {msg.fix_type}")
                    print(f"Satellites: {msg.satellites_visible}")
                    print(f"Latitude: {msg.lat / 1e7}°")
                    print(f"Longitude: {msg.lon / 1e7}°\n")

                elif msg.get_type() == 'BATTERY_STATUS':
                    print(f"Timestamp: {time.time()}")
                    print(f"Battery Voltage: {msg.voltages[0] / 1000.0} V")
                    print(f"Battery Current: {msg.current_battery / 100.0} A")
                    print(f"Battery Remaining: {msg.battery_remaining}%\n")
                
                else :
                    print(f"Message (unknown type): ", msg)
                
                # You can add other message types as needed for more telemetry
                # For example, 'HEARTBEAT', 'SYS_STATUS', 'ATTITUDE', etc.


    except KeyboardInterrupt:
        # Graceful exit on Ctrl+C
        print("\nReceived Ctrl+C, exiting the program...")
        mav.close()  
        sys.exit(0) 

except SerialException: 
    print(f"[{time.strftime("%I:%M:%S %p")}] ✖︎ The port is busy")

