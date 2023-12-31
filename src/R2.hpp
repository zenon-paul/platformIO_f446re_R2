#ifndef R2_INCLUDE
#define R2_INCLUDE

//*スイッチ
//*モーター二つ
//*センサー二つ
//*エンコーダー二つ
//*サーボ二つ
//*スイッチ一つ
//83 で90
#define MOTIONSIZE 100
#define SLOW_DUTY_R 0.2
#define SLOW_DUTY_L 0.225
#define SLOW_DUTY_R_BACK 0.2
#define SLOW_DUTY_L_BACK 0.2
#define BUFFER_SIZE 64
#define RATE 115200
#define ARC_RAD (0.5*PI)
#define L 90


//80度指定で90度曲がる
enum{GO,BACK,CLOCK,ANTI,SWWAIT,OPENARM,R2SLEEP,SENCER_G,SENCER_B,AXLG,AXLB,AXRG,AXRB,MOTIONTYPES};
//   0   1    2     3     4       5      6         7      8         9   10   11   12   13    
#include<mbed.h>
#include"motor.hpp"
#include"sencer.hpp"
#include"encoder.hpp"
#include"arm.hpp"

asm(".global _printf_float");
using ThisThread::sleep_for;

typedef struct type_motion{
    int index;
    int argu;
}t_motion;

void InitR2();
int  R2MakeMotionList(int m,int* indlist,int* arglist);
void R2Simulation();
void R2Go(int mm);

#endif
