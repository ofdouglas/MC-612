import serial
import time
import math

def serial_reader(ser):
    f = open('logfile', 'w')
    while ser.in_waiting > 0:
        line = ser.readline
        if line[0] == '#':
            f.write(line[1:end])
    

ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)
time.sleep(0.1)

# This is a poor bug-fix: a null character is mysteriously found in the
# buffer when the motor controller comes online, causing the first
# line received to be discarded as junk
ser.write('\n'.encode('utf-8'))

ser.write('tlg\n')
ser.write('pos=90\n')
time.sleep(1.5)
ser.write('pos=180\n')
time.sleep(1.5)
ser.write('pos=270\n')
time.sleep(1.5)
ser.write('pos=0\n')

speeds = [25, 50, 100, 200]

for i in range(len(speeds)):
    ser.write('rpm='+str(speeds[i])+'\n'.encode('utf-8'))
    time.sleep(2)
    ser.write('rpm=-'+str(speeds[i])+'\n'.encode('utf-8'))
    time.sleep(2)
    
print ser.write('brk\n'.encode('utf-8'))
ser.close()
