import serial
import time
import glob

# Configure your serial connection
# Auto-detect USB serial port
usb_ports = glob.glob('/dev/cu.usb*')
if not usb_ports:
    raise Exception("No USB serial ports found matching /dev/cu.usb*")
port = usb_ports[0]  # Use the first match
if len(usb_ports) > 1:
    print(f"Multiple USB ports found: {usb_ports}. Using {port}")

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
        if response: # Ensure response is not an empty string
            print(f"{label}: {baud_rates[int(response)]}")
        else:
            print(f"{label}: Invalid/No Response")
    elif label == 'API Mode': 
        if response: # Ensure response is not an empty string
            print(f"{label}: {api_modes[int(response)]}")
        else:
            print(f"{label}: Invalid/No Response")
    else:
        print(f"{label}: {response}")

# Write changes to flash and exit
s.write(b'ATWR\r')
print("Write to flash:", s.read(20).decode(errors='ignore').strip())

s.write(b'ATCN\r')
print("Exit command mode:", s.read(20).decode(errors='ignore').strip())

s.close()
print("Serial port closed.")
