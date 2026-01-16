import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 9600)  # Adjust port if needed
time.sleep(2)  # Wait for serial connection

ser.write(b'M')  # Start the Arduino's routine
print("Sent: M (Started Mecanum Routine)")
ser.close()
