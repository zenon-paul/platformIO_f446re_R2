#ifndef R2_INCLUDE
#define R2_INCLUDE

//*スイッチ
//*モーター二つ
//*センサー二つ
//*エンコーダー二つ
//*サーボ二つ
//*スイッチ一つ

#define MOTIONSIZE 32
#define SLOW_DUTY 0.2
enum{GO,BACK,CLOCK,ANTI,SWWAIT,OPENARM,R2SLEEP,SLOWBACK,MOTIONTYPES};
//   0   1    2     3     4       5      6         7      8
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

#endif