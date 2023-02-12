import matplotlib.pyplot as plt
import serial

serialPort = serial.Serial(port = "/dev/tty.usbmodem103", baudrate=115200,
                           bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

serialString = ""    # Used to hold data coming over UART

time = 0
xs, ys = [], []

while(1):
    if(serialPort.in_waiting > 0):
        serialString = serialPort.readline()
        data = serialString.decode('Ascii')
        xs.append(time)
        ys.append(float(data.strip()))
        time += 1
        plt.plot(xs, ys, 'b')
        plt.pause(0.05)

plt.show(block=True)



