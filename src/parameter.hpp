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
#define RAD_PULSE (RESOLUTION/(2*PI))//rad * RAD_PULSE == pulse
#define MM_PULSE (RESOLUTION/(2.0*PI*WHEEL_RADIUS))//mm * MM_PULSE == pulse
//-------left_wheel--------
#define MT_L_KP 0.0003 //0.0004
#define MT_L_KI 0.000003
#define MT_L_KD 0
//-------right_wheel--------
#define MT_R_KP 0.0003//ｐゲイン不足
#define MT_R_KI 0.000003
#define MT_R_KD 0
//-------period------------
#define dTs 0.1/*second*/
#define dTms 100/*milisec*/
#define dTus 100000/*microsec*/
//------allowable_error-------
#define ALLOWABLEERROR 0.01//目標の1パーセントの誤差以内で許容
#define SPEESTOLERANCE 10 // n pulse/0.1s で許容
//--------slow---------------
#endif