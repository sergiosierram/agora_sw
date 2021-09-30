import time
import RPi.GPIO as GPIO
import Adafruit_PCA9685

pinResetRight = 7 #gpio4
pinEnableRight = 22 #gpio25

pinResetLeft = 18 #gpio24
pinEnableLeft = 16 #gpio23

GPIO.setmode(GPIO.BOARD)

GPIO.setup(pinResetRight, GPIO.OUT)
GPIO.setup(pinEnableRight, GPIO.OUT)
GPIO.setup(pinResetLeft, GPIO.OUT)
GPIO.setup(pinEnableLeft, GPIO.OUT)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(1000)
pwm.set_pwm(0,0, 2048)
pwm.set_pwm(1,0, 2048)

GPIO.output(pinEnableRight, GPIO.LOW)
GPIO.output(pinEnableRight, GPIO.HIGH)
time.sleep(0.5)
GPIO.output(pinEnableRight, GPIO.LOW)

GPIO.output(pinEnableLeft, GPIO.LOW)
GPIO.output(pinEnableLeft, GPIO.HIGH)
time.sleep(0.5)
GPIO.output(pinEnableLeft, GPIO.LOW)

GPIO.output(pinResetRight, GPIO.LOW)
GPIO.output(pinResetRight, GPIO.HIGH)
time.sleep(0.5)
GPIO.output(pinResetRight, GPIO.LOW)

GPIO.output(pinResetLeft, GPIO.LOW)
GPIO.output(pinResetLeft, GPIO.HIGH)
time.sleep(0.5)
GPIO.output(pinResetLeft, GPIO.LOW)

while True:
    lin = float(input("lin: "))
    ang = float(input("ang: "))
    
    vl =  lin - (0.525/2.0)*ang
    vr =  lin + (0.525/2.0)*ang
    
    rpm_l = (vl*60/0.525)
    rpm_r = (vr*60/0.525)
    
    pwm_l = 11.372*rpm_l + 2048
    pwm_r = -11.372*rpm_r + 2048
    
    print("left: "+str(pwm_l))
    print("right: "+str(pwm_r))
    
    send = int(input("send?"))
    
    if send==1:
        pwm.set_pwm(0,0, int(pwm_r))
        pwm.set_pwm(1,0, int(pwm_l))
        time.sleep(0.1)   
