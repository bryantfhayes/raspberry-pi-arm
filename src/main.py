# THE MAIN FILE TO EXECUTE!!!
# example: from model import file.py
import ctypes
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.OUT)
GPIO.output(4, GPIO.HIGH)

def main():
    I2CManager = ctypes.CDLL('/home/pi/development/raspberry-pi-arm/src/model/I2CManager.so')
    ret = I2CManager.init();
    ret += I2CManager.setServoPwm(1, 1300);
    
    print(ret)

    while True:
    	pass

if __name__ == "__main__":
    main()
