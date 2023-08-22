#include"encoder.hpp"
#include<mbed.h>

Encoder::Encoder(){
}
void Encoder::ENCReset(){
    Count = 0;
    //CountMinus = 0;
    PrevCnt = 0;
    //PrevCntMinus = 0;
}

