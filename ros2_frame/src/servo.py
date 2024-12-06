import time
import RPi.GPIO as GPIO

class MyGPIO():
    init_count=0
    mode = GPIO.BCM

    def __init__(self,idx,init_with_high):
        if MyGPIO.init_count ==0:
            GPIO.setmode(MyGPIO.mode)
        MyGPIO.init_count += 1
        self.idx = idx
        initial = GPI0.HIGH if init_with_high else GPIO.LOW
        GPIO.setup (self.idx,GPIO.OUT,initial=initial)

    def __del__ (self):
            MyGPIO.init_count-=1
            if MyGPIO.init_count==0:
                GPIO.cleanup()

    def high(self):
            GPIO.output(self.idx,GPIO.HIGH)

    def low(self):
            GPIO.output(self.idx,GPIO.LOW)

class MyPWM(MyGPIO):
    def __init__(self,idx,frequency):
        super(MyPWM,self).__init__(idx,init_with_high=False)
        self.pwm=GPIO.PWM(idx,frequency)
        self.pwm.start(0)

    def change_duty_cycle(self,cycle):
        self.pwm.ChangeDutyCycle(cycle)

class Steer(object):
    def __init__(self,idx):
        self.pwm=MyPWM(idx,50)

    def alpha2frequency(self,alpha):
        return alpha * 10.5 / 270. + 1.5 #该舵机度数与占空比转换公式

    def run(self,alpha):
        '''
        占空比为1.5时对应0°  为12时对应270°  
        转速大约为300°/s
        '''
        if alpha>=0 and alpha<=270:
            self.pwm.change_duty_cycle(self.alpha2frequency(alpha))
            time.sleep(alpha/300)
            self.pwm.change_duty_cycle(0)#清除PWM谐波影响
def servo_rotate(alpha):
    steer = Steer(18)#gpio接口固定为BCM编码下的18，板载编码下为12
    steer.run(alpha)