#include<mbed.h>//左右ちゃんと動くが目標パルス数が増えると左右の誤差がでる//次同時に動かす
#include<stdio.h>
#include"R2.hpp"//ピンかぶりに気を付け
#include"parameter.hpp"
#include"motor.hpp"

asm(".global _printf_float");
using ThisThread::sleep_for;
//-motor----------------------
DigitalOut MtLDirpin(D2);//mtldir
DigitalOut MtRDirpin(D7);//mtrdir
PwmOut MtLSpdpin(D9);
PwmOut MtRSpdpin(D6);
//-enc-------------------------
InterruptIn EncLApin(D4);
InterruptIn EncRApin(D11);
DigitalIn EncLBpin(D3);
DigitalIn EncRBpin(D12);
//-sencer------------------------
//DigitalIn SncLpin(D2);//sencer
//DigitalIn SncRpin(D15);//sencer
//DigitalIn Snc2Lpin(D10);//sencer
//DigitalIn Snc2Rpin(D5);//sencer
//DigitalIn Snc2Lpin(A5);//sencer
//DigitalIn Snc2Rpin(D15);//sencer
//-arm-------------------------
PwmOut ArmServopin(D14);//servo初期値によっては動かないこともあるので注意あとピンがきつきつ
DigitalIn sw(D13);//スイッチに直つなぎ


static UnbufferedSerial send_serial(A0,A1);//tx,rx
DigitalOut led(A4);
t_motion motionlist[MOTIONSIZE];
int motions;
int crrmotion;

MT mtl(LEFT);
MT mtr(RIGHT);
Encoder encl;
Encoder encr;
Arm arm;



Ticker pidfunc;
Ticker sendfunc;
Ticker servofunc;

char send_data[BUFFER_SIZE];
int ii = 0;

double f(int x){
    return ANGLE*x + SLICE;
}

void ServoCL(){
    if(ii>90){
        return;
    }
    ArmServopin.pulsewidth_us(550+ii*10);
    ii++;
}

void ServoOP(){
    if (ii>90){
        return;
    }
    ArmServopin.pulsewidth_us(1450-ii*10);
    ii++;
}

void SendR2Status(){//ticker
    //memset(send_data,'\0',BUFFER_SIZE);
    //sprintf(send_data,"L(gm%d cm%d csm%d )R(gm%d cm%d csm%d )\n",(int)mtl.goal_mm,(int)(((mtl.Direction>0)?encl.Count:-encl.Count)/MM_PULSE),(int)(mtl.spd/MM_PULSE),(int)mtr.goal_mm,(int)(((mtr.Direction>0)?encr.Count:-encr.Count)/MM_PULSE),(int)(mtr.spd/MM_PULSE));
}

void R2MotorOperate(){//ticker
    if(mtl.Mode == PIDCONTROL){
        MtLSpdpin = mtl.PID((mtl.Direction>0)?encl.Count:-encl.Count);
        MtLDirpin = mtl.Dir;
    }
    else if(mtl.Mode == SLOWBACK){
        MtLSpdpin = SLOW_DUTY_L_BACK;
        MtLDirpin = DIR_PLUS;
    }
    else if(mtl.Mode == SLOW){
        MtLSpdpin = SLOW_DUTY_L;
        MtLDirpin = DIR_MINUS;
    }
    else{
        MtLSpdpin = 0;
    }

    if(mtr.Mode == PIDCONTROL){
        MtRSpdpin = mtr.PID((mtr.Direction>0)?encr.Count:-encr.Count);
        MtRDirpin = mtr.Dir;
    }
    else if(mtr.Mode == SLOWBACK){
        MtRSpdpin = SLOW_DUTY_R_BACK;
        MtRDirpin = DIR_MINUS;
    }
    else if(mtr.Mode == SLOW){
        MtRSpdpin = SLOW_DUTY_R;
        MtRDirpin = DIR_PLUS;
    }
    else{
        MtRSpdpin = 0;
    }

}

void R2ArmClose(){//interrupt//サーボ操作すると右のモーター動かないのでピン変え

    if(arm.Activation == NONACTIVE){
        return;
    }
    led = 1;
    sleep_for(3000ms);
    led = 0;
    servofunc.attach_us(ServoCL,15000);
    R2Go(f(300));
    servofunc.detach();
    
    ii = 0;
    //mtl.Mode = SLOW;
    //mtr.Mode = SLOW;

    /*for(int i = 0;i<=90;i++){
        ArmServopin.pulsewidth_us(550+i*10);
        sleep_for(15ms);
    }*/
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




void R2Go(int mm){
    printf("GO\n");
    mtl.Mode = PIDCONTROL;
    mtr.Mode = PIDCONTROL;

    mtl.make_velo_plan(f(mm));
    mtr.make_velo_plan(f(mm));
    mtl.Direction = ENC_MINUS;
    mtr.Direction = ENC_PLUS;

    while(1){
        if((mtl.period == 3)&&(mtr.period == 3)){
            break;
        }
        printf("L(gm%d cm%d csm%d )R(gm%d cm%d csm%d )\n",(int)mtl.goal_mm,(int)(((mtl.Direction>0)?encl.Count:-encl.Count)/MM_PULSE),(int)(mtl.spd/MM_PULSE),(int)mtr.goal_mm,(int)(((mtr.Direction>0)?encr.Count:-encr.Count)/MM_PULSE),(int)(mtr.spd/MM_PULSE));
    }
    sleep_for(10);
    mtl.Mode = STOP;
    mtr.Mode = STOP;
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();
}
void R2Back(int mm){
    printf("BACK\n");
    mtl.Mode = PIDCONTROL;
    mtr.Mode = PIDCONTROL;

    mtl.make_velo_plan(f(mm));
    mtr.make_velo_plan(f(mm));
    mtl.Direction = ENC_PLUS;
    mtr.Direction = ENC_MINUS;

    while(1){
        if((mtl.period == 3)&&(mtr.period == 3)){
            break;
        }
        printf("L(gm%d cm%d csm%d )R(gm%d cm%d csm%d )\n",(int)mtl.goal_mm,(int)(((mtl.Direction>0)?encl.Count:-encl.Count)/MM_PULSE),(int)(mtl.spd/MM_PULSE),(int)mtr.goal_mm,(int)(((mtr.Direction>0)?encr.Count:-encr.Count)/MM_PULSE),(int)(mtr.spd/MM_PULSE));
    }
    sleep_for(10);
    mtl.Mode = STOP;
    mtr.Mode = STOP;
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();
}
void R2ClockRotation(int deg){
    printf("CLOCK\n");
    mtl.Mode = PIDCONTROL;
    mtr.Mode = PIDCONTROL;

    double rad = deg*DEG_RAD;
    double arc = 0.5*DIRE_WHEELS*rad;

    mtl.make_velo_plan(f(arc));
    mtr.make_velo_plan(f(arc));
    mtl.Direction = ENC_MINUS;
    mtr.Direction = ENC_MINUS;

    while(1){
        if((mtl.period == 3)&&(mtr.period == 3)){
            break;
        }
        printf("L(gm%d cm%d csm%d )R(gm%d cm%d csm%d )\n",(int)mtl.goal_mm,(int)(((mtl.Direction>0)?encl.Count:-encl.Count)/MM_PULSE),(int)(mtl.spd/MM_PULSE),(int)mtr.goal_mm,(int)(((mtr.Direction>0)?encr.Count:-encr.Count)/MM_PULSE),(int)(mtr.spd/MM_PULSE));
    }
    sleep_for(10);
    mtl.Mode = STOP;
    mtr.Mode = STOP;
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();
}
void R2AntiClockRotation(int deg){
    printf("ANTI\n");
    mtl.Mode = PIDCONTROL;
    mtr.Mode = PIDCONTROL;
    
    double rad = deg*DEG_RAD;
    double arc = 0.5*DIRE_WHEELS*rad;

    mtl.make_velo_plan(f(arc));
    mtr.make_velo_plan(f(arc));
    mtl.Direction = ENC_PLUS;
    mtr.Direction = ENC_PLUS;

    while(1){
        if((mtl.period == 3)&&(mtr.period == 3)){
            break;
        }
        printf("L(gm%d cm%d csm%d )R(gm%d cm%d csm%d )\n",(int)mtl.goal_mm,(int)(((mtl.Direction>0)?encl.Count:-encl.Count)/MM_PULSE),(int)(mtl.spd/MM_PULSE),(int)mtr.goal_mm,(int)(((mtr.Direction>0)?encr.Count:-encr.Count)/MM_PULSE),(int)(mtr.spd/MM_PULSE));
    }
    sleep_for(10);
    mtl.Mode = STOP;
    mtr.Mode = STOP;
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();
}

void R2SwitchWait(){
    unsigned int unsintmax = ~0;
    unsigned int checker = 0;
    int flag = 0;
    int interruptflag = 0;

    printf("wait\n");
    mtl.Mode = STOP;
    mtr.Mode = STOP;

    arm.Activation = ACTIVE;
    while(arm.Status == OPENED){
        int swstatus = (sw)?1:0;
        checker = (checker << 1) + swstatus;
        flag = (checker == unsintmax);
        printf("sw:%d fg%d ls%d rs%d\n",swstatus,flag,mtl.Mode,mtr.Mode);
        if(flag == 1){
            R2ArmClose();
        }
    }
    arm.Activation = NONACTIVE;
    mtl.Mode = STOP;
    mtr.Mode = STOP;
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();
    ArmServopin.pulsewidth_us(1450);
    printf("wait_\n");
}

void R2ArmOpen(){
    servofunc.attach_us(ServoOP,15000);
    R2Back(f(350));
    servofunc.detach();
    ii = 0;
    arm.Status = OPENED;
    sleep_for(100);
    ArmServopin.pulsewidth_us(550);
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

void R2SencerGo(int id){
    /*unsigned int sncmax = 4;

    unsigned int snclchecker = 0;
    int snclflag = 0;
    int snclinterruptflag = 0;

    unsigned int sncrchecker = 0;
    int sncrflag = 0;
    int sncrinterruptflag = 0;


    mtl.Mode = SLOW;
    mtr.Mode = SLOW;
    int statusl = 0;
    int statusr = 0;

    if(id == THRESHOLD1){
        while(1){
            if((statusl == 1)&&(statusr == 1)){
                break;
            }
            if(SncLpin == OUT){
                mtl.Mode = STOP;
                statusl = 1;
            }
            if(SncRpin == OUT){
                mtr.Mode = STOP;
                statusr = 1;
            }
            printf("L%d R%d\n",(SncLpin == 1)?1:0,(SncRpin == 1)?1:0);
            //sleep_for(10ms);
        }
    }*/
    /*else if(id == THRESHOLD2){
        while(1){
            if((statusl == 1)&&(statusr == 1)){
                break;
            }
            if(Snc2Lpin == OUT){
                mtl.Mode = STOP;
                statusl = 1;
            }
            if(Snc2Rpin == OUT){
                mtr.Mode = STOP;
                statusr = 1;
            }
            printf("L%d R%d\n",(Snc2Lpin == 1)?1:0,(Snc2Rpin == 1)?1:0);
            sleep_for(10ms);
        }
    }*/
    
    mtl.Mode = STOP;
    mtr.Mode = STOP;
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();
}

void R2SencerBack(int id){
    /*int mask = 1023;



    mtl.Mode = SLOWBACK;
    mtr.Mode = SLOWBACK;
    sleep_for(500ms);
    unsigned int statusl;
    unsigned int statusr;

    if(id == THRESHOLD1){
        statusl = 0;
        statusr = 0;
         while(1){
            if((statusl == 1)&&(statusr == 1)){
                break;
            }
            if(SncLpin == IN){
                mtl.Mode = STOP;
                statusl = 1;
            }
            if(SncRpin == IN){
                mtr.Mode = STOP;
                statusr = 1;
            }
            printf("L%d R%d\n",(SncLpin == 1)?1:0,(SncRpin == 1)?1:0);
            sleep_for(10ms);
        }
    }*/
    /*else if(id == THRESHOLD2){
        while(1){
            if((statusl == 1)&&(statusr == 1)){
                break;
            }
            if(Snc2Lpin == IN){
                mtl.Mode = STOP;
                statusl = 1;
            }
            if(Snc2Rpin == IN){
                mtr.Mode = STOP;
                statusr = 1;
            }
            printf("L%d R%d\n",(Snc2Lpin == 1)?1:0,(Snc2Rpin == 1)?1:0);
            sleep_for(10ms);
        }
    }*/
    
    mtl.Mode = STOP;
    mtr.Mode = STOP;
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();
}

void AxisLeftGo(int deg){//右動く
    mtl.Mode = STOP;
    mtr.Mode = PIDCONTROL;

    double rad = deg*DEG_RAD;
    double arc = DIRE_WHEELS*rad;

    mtr.make_velo_plan(f(arc));
    mtr.Direction = ENC_PLUS;

    while(1){
        if(mtr.period == 3){
            break;
        }
        printf("L(gm%d cm%d csm%d )R(gm%d cm%d csm%d )\n",(int)mtl.goal_mm,(int)(((mtl.Direction>0)?encl.Count:-encl.Count)/MM_PULSE),(int)(mtl.spd/MM_PULSE),(int)mtr.goal_mm,(int)(((mtr.Direction>0)?encr.Count:-encr.Count)/MM_PULSE),(int)(mtr.spd/MM_PULSE));
    }
    sleep_for(10);
    mtr.Mode = STOP;
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();
}
void AxisLeftBack(int deg){//右動く
    printf("BACK\n");
    mtl.Mode = STOP;
    mtr.Mode = PIDCONTROL;

    double rad = deg*DEG_RAD;
    double arc = DIRE_WHEELS*rad;
    mtr.make_velo_plan(f(arc));
    mtr.Direction = ENC_MINUS;

    while(1){
        if(mtr.period == 3){
            break;
        }
        printf("L(gm%d cm%d csm%d )R(gm%d cm%d csm%d )\n",(int)mtl.goal_mm,(int)(((mtl.Direction>0)?encl.Count:-encl.Count)/MM_PULSE),(int)(mtl.spd/MM_PULSE),(int)mtr.goal_mm,(int)(((mtr.Direction>0)?encr.Count:-encr.Count)/MM_PULSE),(int)(mtr.spd/MM_PULSE));
    }
    sleep_for(10);
    mtr.Mode = STOP;
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();
}
void AxisRightGo(int deg){//左
    printf("GO\n");
    mtl.Mode = PIDCONTROL;
    mtr.Mode = STOP;

    double rad = deg*DEG_RAD;
    double arc = DIRE_WHEELS*rad;
    mtl.make_velo_plan(f(arc));
    mtl.Direction = ENC_MINUS;

    while(1){
        if(mtl.period == 3){
            break;
        }
        printf("L(gm%d cm%d csm%d )R(gm%d cm%d csm%d )\n",(int)mtl.goal_mm,(int)(((mtl.Direction>0)?encl.Count:-encl.Count)/MM_PULSE),(int)(mtl.spd/MM_PULSE),(int)mtr.goal_mm,(int)(((mtr.Direction>0)?encr.Count:-encr.Count)/MM_PULSE),(int)(mtr.spd/MM_PULSE));
    }
    sleep_for(10);
    mtl.Mode = STOP;
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();
}
void AxisRightBack(int deg){
    printf("Back\n");
    mtl.Mode = PIDCONTROL;
    mtr.Mode = STOP;

    double rad = deg*DEG_RAD;
    double arc = DIRE_WHEELS*rad;
    mtl.make_velo_plan(f(arc));
    mtl.Direction = ENC_PLUS;

    while(1){
        if(mtl.period == 3){
            break;
        }
        printf("L(gm%d cm%d csm%d )R(gm%d cm%d csm%d )\n",(int)mtl.goal_mm,(int)(((mtl.Direction>0)?encl.Count:-encl.Count)/MM_PULSE),(int)(mtl.spd/MM_PULSE),(int)mtr.goal_mm,(int)(((mtr.Direction>0)?encr.Count:-encr.Count)/MM_PULSE),(int)(mtr.spd/MM_PULSE));
    }
    sleep_for(10);
    mtl.Mode = STOP;
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();
}

void R2Simulation(){
    
    while(crrmotion<motions){
        printf("Simulation:%d\n",crrmotion);
        switch(motionlist[crrmotion].index){
            case GO:
                R2Go(motionlist[crrmotion].argu);
                break;
            case BACK:
                R2Back(motionlist[crrmotion].argu);
                break;
            case CLOCK:
                R2ClockRotation(motionlist[crrmotion].argu);
                break;
            case ANTI:
                R2AntiClockRotation(motionlist[crrmotion].argu);
                break;
            case SWWAIT:
                R2SwitchWait();
                break;
            case OPENARM:
                R2ArmOpen();
                ArmServopin.pulsewidth_us(550);
                break;
            case R2SLEEP:
                R2Sleep(motionlist[crrmotion].argu);
                break;
            case SENCER_G:
                R2SencerGo(motionlist[crrmotion].argu);
                break;
            case SENCER_B:
                R2SencerBack(motionlist[crrmotion].argu);
                break;
            case AXLG:
                AxisLeftGo(motionlist[crrmotion].argu);
                break;
            case AXLB:
                AxisLeftBack(motionlist[crrmotion].argu);
                break;
            case AXRG:
                AxisRightGo(motionlist[crrmotion].argu);
                break;
            case AXRB:
                AxisRightBack(motionlist[crrmotion].argu);
            default:
                break;
        }
        crrmotion++;
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

void InitR2(){
    mtl.MTSetGein(MT_L_KP,MT_L_KI,0);
    mtr.MTSetGein(MT_R_KP,MT_R_KI,0);
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();
    ArmServopin.period_us(20000);  //周期設定20ms
    ArmServopin.pulsewidth_us(550);

    pidfunc.attach_us(R2MotorOperate,dTus);//単位がマイクロ秒割込み開始
    EncLApin.rise(CountEncoderl);
    EncRApin.rise(CountEncoderr);
//-----serial------------------------------
    //send_serial.baud(RATE);
    //send_serial.format(8,SerialBase::None,1);
    //sendfunc.attach(SendR2Status,100ms);//送信ticker割込み
//-----------------------------------------
}
