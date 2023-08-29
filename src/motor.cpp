#include"motor.hpp"
#include"parameter.hpp"

MT::MT(){
//---initialize_mt------
    GoalPulse = 0;
    Direction = 0;
    PrevErr = 0;
    PrevOutPutp = 0;
    PrevOutPutv = 0;
    Acc = 0;
    Dir = DIR_PLUS;
    Speed = 0;
    Mode = STOP;
    T[0] = 0;
    T[1] = 0;
    T[2] = 0;
}

void MT::MTSetGein(double p,double i,double d){
    kp = p;
    ki = i;
    kd = d;
}

void MT::MTReset(){
    GoalPulse = 0;
    Direction = 0;
    PrevErr = 0;
    PrevOutPutp = 0;
    PrevOutPutv = 0;
    Acc = 0;
    Dir = DIR_PLUS;
    Speed = 0;
    Mode = PIDCONTROL;
    T[0] = 0;
    T[1] = 0;
    T[2] = 0;
}
double MT::PID(int Current){
//---位置型------------------------
    int err = GoalPulse - Current;
    Acc += (double)err*dTs;
    double errdif = (double)(err - PrevErr)/dTs;
    PrevErr = err;

    if(err>0){//方向決め
        Dir = (Direction>0)?DIR_PLUS:DIR_MINUS;//ピンに出力
    }
    else{
        Dir = (Direction>0)?DIR_MINUS:DIR_PLUS;//ピンに出力
    }

    double outputp = kp*(double)err + ki*Acc + kd*errdif;

//---速度型------------------------
    double outdif = outputp - PrevOutPutp;
    double outputv = outdif + PrevOutPutv;

    PrevOutPutp = outputp;
    PrevOutPutv = outputv;

    return (outputv>=0)?outputv:-outputv;//ピンに出力
}

void MT::MakeVeloPlan(int mm){
    if(mm<0){
        T[0] = 0;
        T[1] = 0;
        T[2] = 0;
        return;
    }
    double t0 = (MAX_SPEED-START_SPEED)/ACC;
    double t2 = (MAX_SPEED-END_SPEED)/ACC;

    double l0 = 0.5*(START_SPEED+MAX_SPEED)*t0;
    double l2 = 0.5*(MAX_SPEED+END_SPEED)*t2;

    double t1 = (mm - l0 - l2)/MAX_SPEED;
    if(t1<0){
        double max_spd = pow((2.0*ACC*mm + pow(START_SPEED,2.0) + pow(END_SPEED,2.0))/2.0,0.5);
        t0 = (max_spd - START_SPEED)/ACC;
        t2 = (max_spd - END_SPEED)/ACC;
        t1 = 0;
    }
    T[0] = (int)t0;
    T[1] = (int)t1;
    T[2] = (int)t2;
}
