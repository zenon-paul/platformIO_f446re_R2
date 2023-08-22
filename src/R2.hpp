#ifndef R2_INCLUDE
#define R2_INCLUDE

//*スイッチ
//*モーター二つ
//*センサー二つ
//*エンコーダー二つ
//*サーボ二つ
//*スイッチ一つ

#define MOTIONSIZE 32
#define GO 0
#define BACK 1
#define CLOCK 2
#define ANTI 3
#define OPEN 4
#define MOTIONTYPES 5
#define SLOW_DUTY 0.2


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