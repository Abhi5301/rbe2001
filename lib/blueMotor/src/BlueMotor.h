#pragma once

class BlueMotor
{
public:
    BlueMotor();
    void setEffort(int effort);
    void moveTo(long position);
    long getPosition();
    void reset();
    void setup();
    static const int ENCA = 0;
    static const int ENCB = 1;


private:
    void setEffort(int effort, bool clockwise);
    static void isr();
    static void interruptA();
    static void interruptB();
    const int tolerance = 3;
    const int PWMOutPin = 11;
    const int AIN2 = 4;
    const int AIN1 = 13;
};