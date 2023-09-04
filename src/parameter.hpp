#ifndef PARAMETER_INCLUDE
#define PARAMETER_INCLUDE

//未定
//------math-----------------
#define PI 3.141592653589
#define DEG_RAD (PI/180.0)//deg * DEG__RAD == rad
//-----r2--------------------
#define RESOLUTION 2048
#define WHEEL_RADIUS 44.0
#define DIRE_WHEELS (375-30)//mm
#define R2RADIUS (0.5*DIRE_WHEELS)
#define RAD_PULSE (RESOLUTION/(2*PI))//rad * RAD_PULSE == pulse
#define MM_PULSE (RESOLUTION/(2.0*PI*WHEEL_RADIUS))//mm * MM_PULSE == pulse
//-------left_wheel--------
#define MT_L_KP 0.000228 //0.0004
#define MT_L_KI 0//0.00001
#define MT_L_KD 0//0.000004
//-------right_wheel--------
#define MT_R_KP 0.00021//ｐゲイン不足
#define MT_R_KI 0.000008
#define MT_R_KD 0//0.000004
//-------period------------
#define dTms 10/*milisec*/
#define dTs (dTms*0.001)/*second*/
#define dTus (dTms*1000)/*microsec*/
//------allowable_error-------
#define ALLOWABLEERROR 0.01//目標の1パーセントの誤差以内で許容
#define SPEESTOLERANCE 10 // n pulse/0.1s で許容
//--------slow---------------
#define ANGLE 0.91397849462366
#define SLICE 71.505376344086
#endif