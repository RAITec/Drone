#include "Data.hpp"

void Data::setMotors(int m1, int m2, int m3, int m4){
    motor1 = m1;
    motor2 = m2;
    motor3 = m3;
    motor4 = m4;
}

void Data::setAngle(float r, float p, float kR, float kP, int t){
    roll   = r;
    pitch  = p;
    kRoll  = kR;
    kPitch = kP;
    tempo   = t;
}