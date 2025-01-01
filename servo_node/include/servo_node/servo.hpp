#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>

class MyGPIO{
    int idx;
    int initial;
    static int init_count;

    public:
    
    MyGPIO(int idx){
        if(init_count==0)
            wiringPiSetupGpio();
        init_count++;
        idx = idx;
        pinMode(idx,OUTPUT);
    }
    ~MyGPIO(){
        init_count--;
        if(init_count==0)
            pinMode(idx,INPUT);
    }
};

class MyPWM:public MyGPIO{
    int idx;
    public:
    MyPWM(int idx,int frequency):MyGPIO(idx),idx(idx){
        softPwmCreate(idx,0,100);
    }
    void change_duty_cycle(int cycle){
        softPwmWrite(idx,cycle);
    }
};

class Steer{
    MyPWM pwm;
    public:
    Steer(int idx):pwm(idx,50){}
    double alpha2frequency(int alpha){
        return alpha * 21 / 270. + 1.5;
    }

    void run(int alpha){
        if(alpha>=0 && alpha<=360){
            pwm.change_duty_cycle(alpha2frequency(alpha));
            delay(alpha/3*20);
            pwm.change_duty_cycle(alpha2frequency(90));
            delay(alpha/3*20);
            pwm.change_duty_cycle(0);
        }
    }
};

int MyGPIO::init_count=0;

void servo_rotate(int alpha){
    Steer steer(18);//gpio接口固定为BCM编码下的18，板载编码下为12
    steer.run(alpha);
}