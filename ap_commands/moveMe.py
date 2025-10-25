from pymavlink import mavutil
import time
import argparse 

parser = argparse.ArgumentParser()
parser.add_argument("move_x", help="Float distance to move in meters.", default=20)
parser.add_argument("move_y", help="Float distance to move in meters.", default=20)
parser.add_argument("move_z", help="Float distance to move in meters.", default=20)


MAC_IP = "" # blank is for sending commands?  
# MAC_IP = "192.168.1.237" 
UDP_PORT = "14550"
TIMEOUT_S = 5 # seconds 
MOVE_X = int(parser.parse_args().move_x)
MOVE_Y = int(parser.parse_args().move_y)
MOVE_Z = int(parser.parse_args().move_z)


# == CONNECT TO THE AUTOPILOT == 
print(f"[{time.strftime("%I:%M:%S %p")}] … Attempting to make object and connection")
mavObj = mavutil.mavlink_connection(f'udp:{MAC_IP}:{UDP_PORT}', source_system=0)  # or serial:'/dev/ttyACM0', baud=115200
print(f"[{time.strftime("%I:%M:%S %p")}] ✔︎ Connected on {MAC_IP}:{UDP_PORT} ")


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
        # else: print(f"Message found: ", msg.get_type())
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
    0b0000111111000000,                 # type_mask: 
                                        # bits (from LSB): [pos 0-2][vel 3-5][accel 6-8][force 9][yaw 10][yaw_rate 11]
                                        # 0 = use field, 1 = ignore
                                        # https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    MOVE_X, MOVE_Y, MOVE_Z,             # x, y, z positions (front, right, height)
    0, 0, 0,                            # x, y, z velocity (ignored)
    0, 0, 0,                            # acceleration (ignored)
    0, 0                                # yaw, yaw_rate (ignored)
)
print(f"[{time.strftime("%I:%M:%S %p")}] ✔︎ Moving ({MOVE_X}m, {MOVE_Y}m, {MOVE_Z}m)")

# Print the location of the drone as long as it's moving
prev_pos = None
threshold = 0.05  # meters
while True:
    msg = mavObj.recv_match(type='LOCAL_POSITION_NED', blocking=False)
    if msg:
        x, y, z = msg.x, msg.y, msg.z
        pos = (x, y, z)
        if prev_pos is None or any(abs(p - pp) > threshold for p, pp in zip(pos, prev_pos)):
            print(f"[{time.strftime('%I:%M:%S %p')}] Position: ({x:.2f}, {y:.2f}, {z:.2f})")
            prev_pos = pos
            tSinceLastPrint = time.perf_counter()
        if time.perf_counter() - tSinceLastPrint > 1: break
    time.sleep(0.05)

print("No more movement ✔︎")