import serial
import time
#Code from: www.aranacorp.com/en/serial-communication-between-raspberry-pi-and-arduino/
if __name__ == "__main__":
    print("Starting serial communication. \nType Ctrl+C to stop")
    with serial.Serial("/dev/ttyACM0", 9600, timeout=1) as arduino: #Assign the arduino to a serial port
        time.sleep(0.05) #wait for serial to open
        if arduino.isOpen():
            print("{} connected!".format(arduino.port))
            try:
                while True: #Main loop
                    dataOut = input("Write message: ")
                    arduino.write(dataOut.encode()) #Encodes into proper protocol
                    while arduino.inWaiting() == 0: pass #inWaiting shows number of bytes waiting in the input buffer
                    if arduino.inWaiting() > 0:
                        dataIn = arduino.readline().decode('utf-8').rstrip() #Decode dataIn
                        print(dataIn)
                        arduino.flushInput() #Flush buffer
            except KeyboardInterrupt:
                print("Ctrl+C pressed, quitting...")
