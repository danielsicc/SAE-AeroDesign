#include <Servo.h>

Servo AA,AB,B,C,D;
int channel1=2,channel2=3,channel3=4,channel4=5,channel5=6,ch1=0,ch2=0,ch3=0,ch4=0,ch5=0,aa1,aa2,bb,cc,dd;

void setup(){
    Serial.begin(9600);
    AA.attach(9);
    AB.attach(10);
    B.attach(11);
    C.attach(12);
    D.attach(13);
    pinMode(channel1,INPUT);
    pinMode(channel2,INPUT);
    pinMode(channel3,INPUT);
    pinMode(channel4,INPUT);
    pinMode(channel5,INPUT);
}

void loop(){
    ch5=pulseIn(channel5,HIGH);
    Serial.println(ch5); Serial.println();
    if(ch5>1900){
        ch1=pulseIn(channel1,HIGH);
        ch2=pulseIn(channel2,HIGH);
        ch3=pulseIn(channel3,HIGH);
        ch4=pulseIn(channel4,HIGH);     
        Serial.println(ch1);
        Serial.println(ch2); 
        Serial.println(ch3);
        Serial.println(ch4);
        Serial.println(ch5); Serial.println();
        if(ch1<1450||ch1>1550){
            aa1=map(ch1,1000,2000,0,180);
            AA.write(aa1);
            aa2=map(ch1,1000,2000,180,0);
            AB.write(aa2);
            //delay(50);
        }
        else{
            AA.write(90);
            AB.write(90);    
        }
        if(ch2<1450||ch2>1550){
            bb=map(ch2,1000,2000,0,180);
            B.write(bb);
            //delay(50);
        }
        else{
            B.write(90);
        }
        if(ch3>1050){
            //cc=map(ch3,1000,2000,0,180);
            C.writeMicroseconds(ch3);
            //delay(50);
        }
        else{
            C.write(90);
        }
        if(ch4<1450||ch4>1550){
            dd=map(ch4,1000,2000,0,180);
            D.write(dd);
            //delay(50);
        }
        else{
            D.write(90);
        }
    }
}
