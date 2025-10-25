from pymavlink import mavutil
import time
import argparse 

parser = argparse.ArgumentParser()
parser.add_argument("spin", help="Distance to rotate.", default=20)
parser.add_argument("spin_rate", help="speed to rotate.", default=20)


MAC_IP = "" # blank is for sending commands?  
# MAC_IP = "192.168.1.237" 
UDP_PORT = "14550"
TIMEOUT_S = 5 # seconds 
SPIN = int(parser.parse_args().spin)
SPIN_RATE = int(parser.parse_args().spin_rate)


# == CONNECT TO THE AUTOPILOT == 
print(f"[{time.strftime("%I:%M:%S %p")}] … Attempting to make object and connection")
mavObj = mavutil.mavlink_connection(f'udp:{MAC_IP}:{UDP_PORT}', source_system=0)  # or serial:'/dev/ttyACM0', baud=115200
print(f"[{time.strftime("%I:%M:%S %p")}] ✔︎ Connected on {MAC_IP}:{UDP_PORT}")


# === WAIT FOR HEARTBEAT ===
print(f"[{time.strftime("%I:%M:%S %p")}] … Waiting for heartbeat...")
start_time = time.time()
heartbeat_received = False

while not heartbeat_received:
    msg = mavObj.recv_match(blocking=False) 
    # this method of checking for heartbeat is more robust. 
    # normally we would check through mavObj.recv_match(type='HEARTBEAT', blocking=True, timeout=2), but we want to
    # be sure that we don't miss the heartbeat by filtering on our own. idk. chat
    if msg:
        if msg.get_type() == 'HEARTBEAT':
            print(f"[{time.strftime("%I:%M:%S %p")}] ✔︎ Heartbeat received from system {msg.get_srcSystem()} component {msg.get_srcComponent()}")
            heartbeat_received = True
        # else: print(f"[{time.strftime("%I:%M:%S %p")}] Message found: ", msg.get_type())
    elif time.time() - start_time > TIMEOUT_S:
        raise RuntimeError(f"[{time.strftime("%I:%M:%S %p")}] ✖︎ No heartbeat received within {TIMEOUT_S} seconds.")
    else:
        time.sleep(0.01)  # small sleep to avoid busy loop


# Move meters relative to drone (BODY_NED)
# X = forward, Y = right, Z = down
mavObj.mav.set_position_target_local_ned_send(
    int(time.time() * 1000) % 4294967295, # time_boot_ms
    mavObj.target_system,
    mavObj.target_component,
    mavutil.mavlink.MAV_FRAME_BODY_NED, # body-relative
    0b000111111000,                 # type_mask:
                                    # bit1:PosX, bit2:PosY, bit3:PosZ, bit4:VelX, bit5:VelY, bit6:VelZ, bit7:AccX, bit8:AccY, bit9:AccZ, bit11:yaw, bit12:yaw rate
                                    # 0 = use field, 1 = ignore
                                    # https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    0, 0, 0,                            # x, y, z positions (front, right, height)
    0, 0, 0,                            # x, y, z velocity (ignored)
    0, 0, 0,                            # acceleration (ignored)
    SPIN, SPIN_RATE                     # yaw, yaw_rate (ignored)
)
print(f"[{time.strftime("%I:%M:%S %p")}] ✔︎ Spinning {SPIN} radians at {SPIN_RATE} radians per unit (?)")

# Print the location of the drone as long as it's moving
# prev_x = None
# while True:
#     msg = mavObj.recv_match(type='LOCAL_POSITION_NED', blocking=False)
#     if msg:
#         x, y, z = msg.x, msg.y, msg.z
#         # only print if the position changed
#         if prev_x is None or abs(prev_x - x) > 0:  # small threshold to reduce spam
#             print(f"[{time.strftime("%I:%M:%S %p")}] Position: ({x:.2f}, {y:.2f}, {z:.2f})")
#             prev_x = x
#         else: break
#     time.sleep(1)

# print(f"[{time.strftime("%I:%M:%S %p")}] No more movement ✔︎")
