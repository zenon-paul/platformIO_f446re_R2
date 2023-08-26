#ifndef MOTOR_INCLUDE
#define MOTOR_INCLUDE
#include<mbed.h>

#define DIR_PLUS 1
#define DIR_MINUS 0
enum {PIDCONTROL,SLOW,STOP};

asm(".global _printf_float");
using ThisThread::sleep_for;

class MT{
    private:
    public:
        int GoalPulse;//目標パルス数(絶対値)//
        int Direction;//前進方向に進んだときエンコーダーを加算/後進方向に進んだときエンコーダーを加算//
        //double OutPutv;//PID速度型の出力値(最終結果PWMに突っ込む値)
        int Dir;//各モーターの回転方向
        int PrevErr;//前回の差分//
        double PrevOutPutp;//前回のPID位置型の出力値//
        double PrevOutPutv;//前回のPID速度型の出力値//
        double Acc;//累積和//

        double kp;
        double ki;
        double kd;

        int Speed;
        int Mode;
        MT();
        void MTSetGein(double p,double i,double d);
        void MTReset();
        double PID(int Current);
};

#endif