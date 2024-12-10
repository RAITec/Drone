#ifndef DATA_HPP
#define DATA_HPP

struct Data{
    int motor1, motor2, motor3, motor4, tempo;

    float roll, pitch, kRoll, kPitch;

    void setMotors(int m1, int m2, int m3, int m4);

    void setAngle(float r, float p, float kR, float kP, int t);
  
};


#endif
