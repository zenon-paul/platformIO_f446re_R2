#include"motor.hpp"
#include"parameter.hpp"

MT::MT(){
//---initialize_mt------
    GoalPulse = 0;
    PlusMinus = 0;
    PrevErr = 0;
    PrevOutPutp = 0;
    PrevOutPutv = 0;
    Acc = 0;
    Dir = DIR_PLUS;
    Speed = 0;
    Mode = PIDCONTROL;
}

void MT::MTSetGein(double p,double i,double d){
    kp = p;
    ki = i;
    kd = d;
}

void MT::MTReset(){
    GoalPulse = 0;
    PlusMinus = 0;
    PrevErr = 0;
    PrevOutPutp = 0;
    PrevOutPutv = 0;
    Acc = 0;
    Dir = DIR_PLUS;
    Speed = 0;
    Mode = PIDCONTROL;
}
double MT::PID(int Current){
//---位置型------------------------
    int err = GoalPulse - Current;
    Acc += (double)err*dTs;
    double errdif = (double)(err - PrevErr)/dTs;
    PrevErr = err;

    if(err>0){//方向決め
        Dir = (PlusMinus>0)?DIR_PLUS:DIR_MINUS;//ピンに出力
    }
    else{
        Dir = (PlusMinus>0)?DIR_MINUS:DIR_PLUS;//ピンに出力
    }

    double outputp = kp*(double)err + ki*Acc + kd*errdif;

//---速度型------------------------
    double outdif = outputp - PrevOutPutp;
    double outputv = outdif + PrevOutPutv;

    PrevOutPutp = outputp;
    PrevOutPutv = outputv;

    return (outputv>=0)?outputv:-outputv;//ピンに出力
}