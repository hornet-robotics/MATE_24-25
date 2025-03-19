import serial
import time

# on pi will be something like '/dev/ttyACM0'
# on Windows see top left of ide (ex: COM5)
# run with sudo to access port
arduino_port = 'COM7' #/dev/ttyACM0'
baud_rate = 9600

# 23 bytes of data (1 byte per char in utf-8 encoding of strings)
# char travels in 10-bit data frame so: 23 bytes * 10 bits (start (1), data (8), end (1)) =  230 bits
# if data needs to be sent once per sec then: 230 baud
# # if data needs to be sent 10 times per sec then: 2300 baud
# use: baud rate >= 2300

#             m0  m1  m2  m3  m4  m5
low_range = "000 000 000 000 000 000"
high_range = "999 999 999 999 999 999"
all_stop = "499 499 499 499 499 499"

#             m0  m1  m2  m3  m4  m5
test_data = "555 555 150 050 122 255"


ser = serial.Serial(arduino_port, baud_rate)
time.sleep(2)

try:

    while True:
        data_to_send = test_data + "\n"  # Include "\r" for end of line
        ser.write(data_to_send.encode('utf-8'))
        print(f"Sent: {data_to_send}")

except KeyboardInterrupt:
    print("Stopping communication.")
    ser.close()