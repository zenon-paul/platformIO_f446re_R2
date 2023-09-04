#include"motor.hpp"
#include"parameter.hpp"
#include<math.h>

MT::MT(int name){
    goal_mm = 0;
    Mode = STOP;
    Direction = 1;
    T[0] = 0;
    T[1] = 0;
    T[2] = 0;
    period = 0;
    count_time = 0;

    prcnt = 0;
    spd = 0;//明日単位変える
    prev_errspd = 0;
    Dir = 0;

    restgoalspd_mm = MIDLE*ACC;
    goalspd_mm = 0;//mm

    errspd = 0;

    acc = 0;
    dif = 0;
    dif_ = 0;

    output_p = 0;
    output_v = 0;
    acc_output_v = 0;
    prev_output_v = 0;
    prev_output_p = 0;

    NAME = name;

}

void MT::MTSetGein(double p,double i,double d){
    kp = p;
    ki = i;
    kd = d;
}

void MT::MTReset(){
    goal_mm = 0;
    Direction = 1;
    T[0] = 0;
    T[1] = 0;
    T[2] = 0;
    period = 0;
    count_time = 0;

    prcnt = 0;
    spd = 0;//明日単位変える
    prev_errspd = 0;
    Dir = 0;

    restgoalspd_mm = MIDLE*ACC;
    goalspd_mm = 0;//mm

    errspd = 0;

    acc = 0;
    dif = 0;
    dif_ = 0;

    output_p = 0;
    output_v = 0;
    acc_output_v = 0;
    prev_output_v = 0;
    prev_output_p = 0;
}



double MT::PID(int cnt){
    
    if(period == 3){
        return 0;
    }
    if(count_time == T[period]){
        count_time = 0;
        period++;
        if(period == 1){
            if(T[period] == 0){
                period = 2;
            }
        }
    }
//------------------pid_posi-----------------------------
    spd = cnt - prcnt;
    prcnt = cnt;
//-----------------------------------------------------------
    if(count_time%MIDLE == 0){
        goalspd_mm = restgoalspd_mm;
    }
    errspd = (int)(goalspd_mm*MM_PULSE) - spd;
    acc += (double)errspd*dTs;
    dif = (double)(errspd-prev_errspd)/dTs;

    prev_errspd = errspd;
    
    //output_p = kp*(double)errspd + ki*acc + kd*dif;
    output_p = kp*(double)errspd + (((NAME == RIGHT)&&(Mode == 2))?0:ki)*acc + kd*dif;
//------------------pid_velo--------------------------

    dif_ = (double)(output_p - prev_output_p)/dTs;
    output_v = dif_ + prev_output_v;

    prev_output_p = output_p;
    prev_output_v = output_v;

    acc_output_v += output_v*dTs;
    
    Dir = (Direction > 0)?1:0;//

    if(period == 0){
        restgoalspd_mm+=ACC;
    }
    else if(period == 2){
        restgoalspd_mm-=ACC;
    }
    count_time++;
    return acc_output_v;
}

//-----------------------------------------------------------

void MT::make_velo_plan(double mm){
    if(mm<0){
        T[0] = 0;
        T[1] = 0;
        T[2] = 0;
        return;
    }
    goal_mm = mm;
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
