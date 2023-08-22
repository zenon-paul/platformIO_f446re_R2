#ifndef ARM_INCLUDE
#define ARM_INCLUDE
#include<mbed.h>

#define CLOSE_PERIOD 500//1450
#define OPEN_PERIOD 500
#define OPENED 1
#define CLOSED 0
#define ACTIVE 1
#define NONACTIVE 0

asm(".global _printf_float");
using ThisThread::sleep_for;


class Arm{
    private:
    public:
        int Status;
        int Activation;
        Arm();
};
#endif