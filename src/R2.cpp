#include<mbed.h>//左右ちゃんと動くが目標パルス数が増えると左右の誤差がでる//次同時に動かす
#include<stdio.h>
#include"R2.hpp"//ピンかぶりに気を付け
#include"parameter.hpp"
#include"motor.hpp"
#define RATE 115200

asm(".global _printf_float");
using ThisThread::sleep_for;
//-motor----------------------
DigitalOut MtLDirpin(D8);//mtldir
DigitalOut MtRDirpin(D7);//mtrdir
PwmOut MtLSpdpin(D9);
PwmOut MtRSpdpin(D6);
//-enc-------------------------
InterruptIn EncLApin(D4);
InterruptIn EncRApin(D11);
DigitalIn EncLBpin(D3);
DigitalIn EncRBpin(D12);
//-sencer------------------------
DigitalIn SncLpin(D2);//sencer
DigitalIn SncRpin(D15);//sencer
DigitalIn Snc2Lpin(D10);//sencer
DigitalIn Snc2Rpin(D5);//sencer
//-arm-------------------------
PwmOut ArmServopin(D14);//servo初期値によっては動かないこともあるので注意あとピンがきつきつ
DigitalIn sw(D13);//スイッチに直つなぎ


static UnbufferedSerial send_serial(A0,A1);//tx,rx
t_motion motionlist[MOTIONSIZE];
int motions;
int crrmotion;

MT mtl;
MT mtr;
Encoder encl;
Encoder encr;
Arm arm;

unsigned int unsintmax = ~0;
unsigned int checker = 0;
int flag = 0;
int interruptflag = 0;

Ticker pidfunc;
Ticker sendfunc;
char send_data[BUFFER_SIZE];

void SendR2Status(){//ticker
    memset(send_data,'\0',BUFFER_SIZE);
    sprintf(send_data,"mvs%d crr%d crrmv%d L(C:%d E:%d S:%d)R(C:%d E:%d S:%d)\n",motions,crrmotion,motionlist[crrmotion],encl.Count,mtl.PrevErr,mtl.Speed,encr.Count,mtr.PrevErr,mtr.Speed);
    send_serial.write(send_data,BUFFER_SIZE);
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

    /*MtLSpdpin = mtl.PID((mtl.Direction>0)?encl.Count:-encl.Count);
    MtLDirpin = mtl.Dir;
    MtRSpdpin = mtr.PID((mtr.Direction>0)?encr.Count:-encr.Count);
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

void R2ArmClose(){//interrupt//サーボ操作すると右のモーター動かないのでピン変え

    if(arm.Activation == NONACTIVE){
        return;
    }

    mtl.Mode = SLOW;
    mtr.Mode = SLOW;

    sleep_for(3000ms);
    for(int i = 0;i<=90;i++){
        ArmServopin.pulsewidth_us(550+i*10);
        sleep_for(15ms);
    }
    arm.Status = CLOSED;

    mtl.Mode = STOP;
    mtr.Mode = STOP;
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
    printf("GO\n");
    mtl.Mode = PIDCONTROL;
    mtr.Mode = PIDCONTROL;

    int goal = (int)(mm*MM_PULSE);
    mtl.GoalPulse = goal;
    mtr.GoalPulse = goal;
    mtl.Direction = ENC_MINUS;
    mtr.Direction = ENC_PLUS;

    while(JudgeConvergence(mtl)||JudgeConvergence(mtr)){//打ち切るか切らないかチェック
        printf("Go L(C:%d E:%d S:%d) R(C:%d E:%d S:%d)\n",encl.Count,mtl.PrevErr,mtl.Speed,encr.Count,mtr.PrevErr,mtr.Speed);
    }
    sleep_for(4000);
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();

    mtl.Mode = STOP;
    mtr.Mode = STOP;
}
void R2Back(int mm){
    mtl.Mode = PIDCONTROL;
    mtr.Mode = PIDCONTROL;

    int goal = (int)(mm*MM_PULSE);
    mtl.GoalPulse = goal;
    mtr.GoalPulse = goal;
    mtl.Direction = ENC_PLUS;
    mtr.Direction = ENC_MINUS;

    while(JudgeConvergence(mtl)||JudgeConvergence(mtr)){//打ち切るか切らないかチェック
        printf("Go L(C:%d E:%d S:%d) R(C:%d E:%d S:%d)\n",encl.Count,mtl.PrevErr,mtl.Speed,encr.Count,mtr.PrevErr,mtr.Speed);
    }
    sleep_for(4000);
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();

    mtl.Mode = STOP;
    mtr.Mode = STOP;
}
void R2ClockRotation(int deg){
    printf("CLOCK\n");
    mtl.Mode = PIDCONTROL;
    mtr.Mode = PIDCONTROL;

    double rad = deg*DEG_RAD;
    double arc = 0.5*DIRE_WHEELS*rad;
    int goal = (int)(arc * MM_PULSE);

    mtl.GoalPulse = goal;
    mtr.GoalPulse = goal;
    mtl.Direction = ENC_MINUS;
    mtr.Direction = ENC_MINUS;

    while(JudgeConvergence(mtl)||JudgeConvergence(mtr)){//打ち切るか切らないかチェック
        printf("Go L(C:%d E:%d S:%d) R(C:%d E:%d S:%d)\n",encl.Count,mtl.PrevErr,mtl.Speed,encr.Count,mtr.PrevErr,mtr.Speed);
    }
    sleep_for(4000);
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();
    
    mtl.Mode = STOP;
    mtr.Mode = STOP;
}
void R2AntiClockRotation(int deg){
    printf("ANTI\n");
    mtl.Mode = PIDCONTROL;
    mtr.Mode = PIDCONTROL;
    
    double rad = deg*DEG_RAD;
    double arc = 0.5*DIRE_WHEELS*rad;
    int goal = (int)(arc * MM_PULSE);

    mtl.GoalPulse = goal;
    mtr.GoalPulse = goal;
    mtl.Direction = ENC_PLUS;
    mtr.Direction = ENC_PLUS;

    while(JudgeConvergence(mtl)||JudgeConvergence(mtr)){//打ち切るか切らないかチェック
        printf("Go L(C:%d E:%d S:%d) R(C:%d E:%d S:%d)\n",encl.Count,mtl.PrevErr,mtl.Speed,encr.Count,mtr.PrevErr,mtr.Speed);
    }
    sleep_for(4000);
    mtl.MTReset();
    mtr.MTReset();
    encl.ENCReset();
    encr.ENCReset();

    mtl.Mode = STOP;
    mtr.Mode = STOP;
}

void R2SwitchWait(){

    printf("wait\n");
    mtl.Mode = STOP;
    mtr.Mode = STOP;

    arm.Activation = ACTIVE;
    while(arm.Status == OPENED){
        int swstatus = (sw)?1:0;
        checker = (checker << 1) + swstatus;
        flag = (checker == unsintmax);
        printf("s:%d f%d ls%d rs%d\n",swstatus,flag,mtl.Mode,mtr.Mode);
        if(flag == 1){
            R2ArmClose();
        }
    }
    arm.Activation = NONACTIVE;

    printf("wait_\n");
}

void R2ArmOpen(){
    printf("open\n");
    sleep_for(5000ms);
    mtl.Mode = SLOWBACK;
    mtr.Mode = SLOWBACK;
    
    mtl.Dir = DIR_PLUS;
    mtr.Dir = DIR_MINUS;

    for(int i = 0;i<=90;i++){
        ArmServopin.pulsewidth_us(1450-i*10);
        sleep_for(15ms);
    }
    arm.Status = OPENED;
    sleep_for(500ms);

    mtl.Mode = STOP;
    mtr.Mode = STOP;
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

void R2SencerBack(int id){
    mtl.Mode = SLOWBACK;
    mtr.Mode = SLOWBACK;
    int statusl = 0;
    int statusr = 0;

    if(id == THRESHOLD1){
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
    }
    else if(id == THRESHOLD2){
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
    }
    
    mtl.Mode = STOP;
    mtr.Mode = STOP;
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
                break;
            case R2SLEEP:
                R2Sleep(motionlist[crrmotion].argu);
                break;
            case SENCER:
                R2SencerBack(motionlist[crrmotion].argu);
                break;
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

    pidfunc.attach_us(R2MotorOperate,dTus);//単位がマイクロ秒割込み開始
    EncLApin.rise(CountEncoderl);
    EncRApin.rise(CountEncoderr);
//-----serial------------------------------
    send_serial.baud(RATE);
    send_serial.format(8,SerialBase::None,1);
    sendfunc.attach(SendR2Status,100ms);//送信ticker割込み
//-----------------------------------------
}
