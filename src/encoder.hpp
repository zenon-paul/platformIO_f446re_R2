#ifndef ENCODERE_INCLUDE
#define ENCODERE_INCLUDE
#include<mbed.h>

#define ENC_PLUS 1
#define ENC_MINUS -1

asm(".global _printf_float");
using ThisThread::sleep_for;

class Encoder{
    private:
    public:
        int Count;
        //int CountMinus;

        int PrevCnt;
        //int PrevCntMinus;

        Encoder();
        void ENCReset();
        //void CountEncoder();
};
#endif