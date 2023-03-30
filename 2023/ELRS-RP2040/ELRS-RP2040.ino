#include <CrsfSerial.h>
#include <Servo.h>
#include <Wire.h>

SerialPIO Receiver(10, 11);
CrsfSerial crsf(Receiver, 200000);
Servo Aileron, Elevator, Motor, Rudder;

void setup(){
    Receiver.begin(200000);
    Serial.begin(9600);
    Aileron.attach(29);
    Elevator.attach(28);
    Motor.attach(27);
    Rudder.attach(26);
}

void loop(){
    crsf.loop();
    Aileron.writeMicroseconds(crsf.getChannel(1));
    Elevator.writeMicroseconds(crsf.getChannel(2));
    Motor.writeMicroseconds(crsf.getChannel(3));
    Rudder.writeMicroseconds(crsf.getChannel(4));
}
