#include<mbed.h>//左右ちゃんと動くが目標パルス数が増えると左右の誤差がでる//次同時に動かす
#include<stdio.h>
#include"R2.hpp"
#include"parameter.hpp"
//#include"pid_control.hpp"
//interruptin の人たちのスタートのタイミング考える
#include"motor.hpp"

asm(".global _printf_float");
using ThisThread::sleep_for;
//-motor----------------------
DigitalOut MtLDirpin(D8);//mtldir
DigitalOut MtRDirpin(D7);//mtrdir
PwmOut MtLSpdpin(D9);
PwmOut MtRSpdpin(D6);
//-enc-------------------------
InterruptIn EncLApin(D4);
InterruptIn EncRApin(D12);
DigitalIn EncLBpin(D3);
DigitalIn EncRBpin(D13);
//-sencer------------------------
DigitalIn SncLpin(D2);//sencer
DigitalIn SncRpin(D0);//sencer
//-arm-------------------------
PwmOut ArmServolpin(D5);//servo
InterruptIn ArmSwpin(D10);//switch


t_motion motionlist[MOTIONSIZE];
int motions;

MT mtl;
MT mtr;
Encoder encl;
Encoder encr;
Arm arm;

Ticker pidfunc;

void R2MotorOperate(){//ticker
    //PIDvelo(&MtL,EncL,&MtR,EncR);
    if(mtl.Mode == PIDCONTROL){
        MtLSpdpin = mtl.PID((mtl.PlusMinus>0)?encl.Count:-encl.Count);
        MtLDirpin = mtl.Dir;
    }
    else if(mtl.Mode == SLOW){
        if(SncLpin == IN){
            MtLSpdpin = 0;
            mtl.Mode = STOP;
            return;
        }
        MtLSpdpin = SLOW_DUTY;
        MtLDirpin = DIR_PLUS;
    }
    else{
        MtLSpdpin = 0;
    }

    if(mtr.Mode == PIDCONTROL){
        MtRSpdpin = mtr.PID((mtr.PlusMinus>0)?encr.Count:-encr.Count);
        MtRDirpin = mtr.Dir;
    }
    else if(mtr.Mode == SLOW){
        if(SncRpin == IN){
            MtRSpdpin = 0;
            mtr.Mode = STOP;
            return;
        }
        MtLSpdpin = SLOW_DUTY;
        MtLDirpin = DIR_MINUS;
    }
    else{
        MtRSpdpin = 0;
    }

    /*MtLSpdpin = mtl.PID((mtl.PlusMinus>0)?encl.Count:-encl.Count);
    MtLDirpin = mtl.Dir;
    MtRSpdpin = mtr.PID((mtr.PlusMinus>0)?encr.Count:-encr.Count);
    MtRDirpin = mtr.Dir;*/
    //testled = !testled;

    //-speed-record-------------
    int spdl = encl.Count - encl.PrevCnt;
    mtl.Speed = (spdl>=0)?spdl:-spdl;

    encl.PrevCnt = encl.Count;


    int spdr = encr.Count - encr.PrevCnt;
    mtr.Speed = (spdr>=0)?spdr:-spdr;

    encr.PrevCnt = encr.Count;
}

void R2ArmClose(){//interrupt
    if(arm.Activation == NONACTIVE){
        return;
    }
    ArmServolpin = CLOSE_DUTY;
    arm.Status = CLOSED;
}

void CountEncoderl(){//inerrupt
//-left-----------------
    if(EncLBpin.read() == 1){
        encl.Count++;
    }
    else{
        encl.Count--;
    }
}

void CountEncoderr(){//inerrupt
//-right---------------------
    if(EncRBpin.read() == 1){
        encr.Count++;
    }
    else{
        encr.Count--;
    }
}

int JudgeConvergence(MT mt){//1-収束しない　0-収束
    if((mt.Speed <= SPEESTOLERANCE)&&((mt.PrevErr/mt.GoalPulse)<ALLOWABLEERROR)){
        return 0;
    }
    return 1;
}


void R2Go(int mm){
    mtl.Mode = PIDCONTROL;
    mtr.Mode = PIDCONTROL;
    //int goal = (int)(mm/MM_PULSE);
    printf("R2Go\n");
    int goal = mm;
    mtl.GoalPulse = goal;
    mtr.GoalPulse = goal;
    mtl.PlusMinus = ENC_MINUS;
    mtr.PlusMinus = ENC_PLUS;

    while(JudgeConvergence(mtl)||JudgeConvergence(mtr)){//打ち切るか切らないかチェック
        printf("Go L(C:%d E:%d S:%d) R(C:%d E:%d S:%d)\n",encl.Count,mtl.PrevErr,mtl.Speed,encr.Count,mtr.PrevErr,mtr.Speed);
    }
    sleep_for(2000);
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();
    printf("_______R2Go\n");
}
void R2Back(int mm){
    mtl.Mode = PIDCONTROL;
    mtr.Mode = PIDCONTROL;
    printf("R2Back\n");
    //int goal = (int)(mm/MM_PULSE);
    int goal = mm;
    mtl.GoalPulse = goal;
    mtr.GoalPulse = goal;
    mtl.PlusMinus = ENC_PLUS;
    mtr.PlusMinus = ENC_MINUS;

    while(JudgeConvergence(mtl)||JudgeConvergence(mtr)){//打ち切るか切らないかチェック
        printf("Go L(C:%d E:%d S:%d) R(C:%d E:%d S:%d)\n",encl.Count,mtl.PrevErr,mtl.Speed,encr.Count,mtr.PrevErr,mtr.Speed);
    }
    sleep_for(2000);
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();
    printf("__________R2Back\n");
}
void R2ClockRotation(int rad){
    mtl.Mode = PIDCONTROL;
    mtr.Mode = PIDCONTROL;
    //int goal = (int)(mm/MM_PULSE);
    int goal = rad;
    mtl.GoalPulse = goal;
    mtr.GoalPulse = goal;
    mtl.PlusMinus = ENC_MINUS;
    mtr.PlusMinus = ENC_MINUS;

    while(JudgeConvergence(mtl)||JudgeConvergence(mtr)){//打ち切るか切らないかチェック
        printf("Go L(C:%d E:%d S:%d) R(C:%d E:%d S:%d)\n",encl.Count,mtl.PrevErr,mtl.Speed,encr.Count,mtr.PrevErr,mtr.Speed);
    }
    sleep_for(2000);
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();
    printf("R2Clock\n");
}
void R2AntiClockRotation(int rad){
    mtl.Mode = PIDCONTROL;
    mtr.Mode = PIDCONTROL;
    //int goal = (int)(mm/MM_PULSE);
    int goal = rad;
    mtl.GoalPulse = goal;
    mtr.GoalPulse = goal;
    mtl.PlusMinus = ENC_PLUS;
    mtr.PlusMinus = ENC_PLUS;

    while(JudgeConvergence(mtl)||JudgeConvergence(mtr)){//打ち切るか切らないかチェック
        printf("Go L(C:%d E:%d S:%d) R(C:%d E:%d S:%d)\n",encl.Count,mtl.PrevErr,mtl.Speed,encr.Count,mtr.PrevErr,mtr.Speed);
    }
    sleep_for(2000);
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();
    printf("R2Anti\n");
}

void R2SwitchWait(){
    mtl.Mode = STOP;
    mtr.Mode = STOP;

    arm.Activation = ACTIVE;
    while(arm.Status == OPENED){
        printf("wait\n");
    }
    arm.Activation = NONACTIVE;
}

void R2ArmOpen(){
    mtl.Mode = STOP;
    mtr.Mode = STOP;
    ArmServolpin = OPEN_DUTY;
    arm.Status = OPENED;
}

void R2Sleep(int sec){
    mtl.Mode = STOP;
    mtr.Mode = STOP;
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();
    sleep_for(sec);
}

void R2SLOWBack(int id){
    mtl.Mode = SLOW;
    mtr.Mode = SLOW;

    
    mtl.Mode = STOP;
    mtr.Mode = STOP;
}

void R2Simulation(){
    int i = 0;
    while(i<motions){
        printf("Simulation:%d\n",i);
        switch(motionlist[i].index){
            case GO:
                R2Go(motionlist[i].argu);
                break;
            case BACK:
                R2Back(motionlist[i].argu);
                break;
            case CLOCK:
                R2ClockRotation(motionlist[i].argu);
                break;
            case ANTI:
                R2AntiClockRotation(motionlist[i].argu);
                break;
            default:
                break;
        }
        i++;
    }
}
int  R2MakeMotionList(int m,int* indlist,int* arglist){//戻り値実際に受け入れられた動作数
    if(MOTIONSIZE<=m){
        return -1;
    }
    else{
        printf("make\n");
        int cnt = 0;
        for(int i = 0;i<m;i++){
            if((0<=indlist[i])&&(indlist[i]<MOTIONTYPES)){
                motionlist[cnt].index = indlist[i];
                motionlist[cnt].argu = arglist[i];
                cnt++;
            }
        }
        motions = cnt;
        return cnt;
    }
}



int main(){

    mtl.MTSetGein(MT_L_KP,MT_L_KI,0);
    mtr.MTSetGein(MT_R_KP,MT_R_KI,0);
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();

    pidfunc.attach(R2MotorOperate,100ms);//単位がマイクロ秒割込み開始
    EncLApin.rise(CountEncoderl);
    EncRApin.rise(CountEncoderr);

    //ArmSwpin.rise(R2ArmClose);

    int id[MOTIONSIZE] = {0,1,0,1,1,0};
    int arg[MOTIONSIZE] = {3000,2000,2000,2000,2000,1000};
    R2MakeMotionList(6,id,arg);
    R2Simulation();
    
    return 0;
}
