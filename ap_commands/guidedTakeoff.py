from pymavlink import mavutil
from serial import SerialException
import time

MAC_IP = "0.0.0.0" 
# MAC_IP = "192.168.1.237" 
UDP_PORT = "14550"
TIMEOUT_S = 5 # seconds 
SERIAL = True
SERIAL_PORT = "/dev/cu.usbserial-A10KFA7J"
BAUD = "57600"

try: 
    # == CONNECT TO THE AUTOPILOT == 
    print(f"[{time.strftime("%I:%M:%S %p")}] … Attempting to make object and connection")
    if SERIAL: 
        mavObj = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD)
        print(f"[{time.strftime("%I:%M:%S %p")}] ✔︎ Connected on {SERIAL_PORT} @ baud {BAUD}")
    else: 
        mavObj = mavutil.mavlink_connection(f'udp:{MAC_IP}:{UDP_PORT}', source_system=0)  # or serial:'/dev/ttyACM0', baud=115200
        print(f"[{time.strftime("%I:%M:%S %p")}] ✔︎ Connected on {MAC_IP}:{UDP_PORT}")


    # === WAIT FOR HEARTBEAT ===
    # print(f"[{time.strftime("%I:%M:%S %p")}] … Waiting for heartbeat...")
    # start_time = time.time()
    # heartbeat_received = False

    # while not heartbeat_received:
    #     msg = mavObj.recv_msg() 
    #     # this method of checking for heartbeat is more robust. 
    #     # normally we would check through mavObj.recv_match(type='HEARTBEAT', blocking=True, timeout=2), but we want to
    #     # be sure that we don't miss the heartbeat by filtering on our own. idk. chat
    #     if msg:
    #         if msg.get_type() == 'HEARTBEAT':
    #             print(f"[{time.strftime("%I:%M:%S %p")}] ✔︎ Heartbeat received from system {msg.get_srcSystem()} component {msg.get_srcComponent()}")
    #             heartbeat_received = True
    #         # else: print(f"Message found: ", msg.get_type())
    #     elif time.time() - start_time > TIMEOUT_S:
    #         raise RuntimeError(f"[{time.strftime("%I:%M:%S %p")}] ✖︎ No heartbeat received within {TIMEOUT_S} seconds.")
    #     else:
    #         time.sleep(0.01)  # small sleep to avoid busy loop



    # 1) === Set mode to GUIDED (ArduCopter custom_mode index: GUIDED = 4) ===
    GUIDED = 4
    mavObj.mav.set_mode_send(
        mavObj.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # tell AP to use custom_mode
        GUIDED
    )
    print(f"[{time.strftime("%I:%M:%S %p")}] … Guided mode command sent")
    time.sleep(1)
    # Check current mode via heartbeat
    msg = mavObj.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if msg:
        print(f"[{time.strftime("%I:%M:%S %p")}] ✔︎ Current mode:", msg.custom_mode)


    # 2) === Arm ===
    mavObj.mav.command_long_send(
        mavObj.target_system, mavObj.target_component,
        400, 0,   # MAV_CMD_COMPONENT_ARM_DISARM
        1, 0, 0, 0, 0, 0, 0
    )
    print(f"[{time.strftime("%I:%M:%S %p")}] … Arm command sent")
    ack = mavObj.recv_match(type='COMMAND_ACK', blocking=True)
    print(f"[{time.strftime("%I:%M:%S %p")}] {"✔︎" if ack.result == 0 else "✖︎"} ARM ACK:", ack.result)


    # 3) === Takeoff (MAV_CMD_NAV_TAKEOFF = 22) — in GUIDED mode ===
    mavObj.mav.command_long_send(
        mavObj.target_system, mavObj.target_component,
        22, 0,
        0, 0, 0, 0,  # params 1–4 often unused for Copter
        0, 0,        # lat/lon (unused: 0 means current position)
        1           # param7: altitude (meters AGL or AMSL per firmware config)
    )
    print(f"[{time.strftime("%I:%M:%S %p")}] … Takeoff command sent")
    ack = mavObj.recv_match(type='COMMAND_ACK', blocking=True)
    print(f"[{time.strftime("%I:%M:%S %p")}] {"✔︎" if ack.result == 0 else "✖︎"} TAKEOFF ACK: {ack.result}")


    # # 4) === Change speed (MAV_CMD_DO_CHANGE_SPEED = 178) ===
    # mavObj.mav.command_long_send(
    #     mavObj.target_system, mavObj.target_component,
    #     178, 0,
    #     1,    # param1: groundspeed
    #     5.0,  # param2: speed m/s
    #     0,0,0,0,0
    # )
    # ack = mavObj.recv_match(type='COMMAND_ACK', blocking=True)
    # print(f"[{time.strftime("%I:%M:%S %p")}] {"✔︎" if ack.result == 0 else "✖︎"} SPEED ACK:", ack.result)

except SerialException: 
    print(f"[{time.strftime("%I:%M:%S %p")}] ✖︎ The port is busy")
