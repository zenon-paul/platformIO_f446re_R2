#ifndef SENCER_INCLUDE
#define SENCER_INCLUDE

#define OUT 0//まだ近づける
#define IN 1//範囲内に入った
#define THRESHOLD1 1
#define THRESHOLD2 2

asm(".global _printf_float");

class Sencer{
    private:
    public:
        int Status;
        Sencer();
    //int OnOff();
};
#endif