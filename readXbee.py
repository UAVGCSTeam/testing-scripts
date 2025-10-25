import serial
import time

# Configure your serial connection
port = '/dev/cu.usbserial-A10KFA7J'
baud = 57600
s = serial.Serial(port, baud, timeout=1)

print(f"Opened serial port {port} at {baud} baud.")
time.sleep(1)

# Enter command mode
s.write(b'+++')
time.sleep(1)
print("Entering command mode:", s.read(10).decode(errors='ignore').strip())

# Query parameters
commands = {
    "ATID": "PAN ID",
    "ATCM": "Channel Mask",
    "ATDH": "Destination High",
    "ATDL": "Destination Low",
    # "ATMY": "My Address",
    "ATBD": "Baud Rate",
    "ATAP": "API Mode"
}

baud_rates = {
    0 : 1200,
    1 : 2400,
    2 : 4800,
    3 : 9600,
    4 : 19200,
    5 : 38400,
    6 : 57600,
    7 : 115200,
    8 : 230400
}

api_modes = {
    0 : "Transparent Mode [0]",
    1 : "API Mode Without Escapes [1]",
    2 : "API Mode With Escapes [2]",
}

for cmd, label in commands.items():
    s.write(f"{cmd}\r".encode())
    time.sleep(0.2)
    response = s.read(20).decode(errors='ignore').strip()
    if label == 'Baud Rate': 
        print(f"{label}: {baud_rates[int(response)]}")
    elif label == 'API Mode': 
        print(f"{label}: {api_modes[int(response)]}")
    else:
        print(f"{label}: {response}")

# Write changes to flash and exit
s.write(b'ATWR\r')
print("Write to flash:", s.read(20).decode(errors='ignore').strip())

s.write(b'ATCN\r')
print("Exit command mode:", s.read(20).decode(errors='ignore').strip())

s.close()
print("Serial port closed.")
