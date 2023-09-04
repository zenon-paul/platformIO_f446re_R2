#ifndef MOTOR_INCLUDE
#define MOTOR_INCLUDE

#define DIR_PLUS 1
#define DIR_MINUS 0

#define ACC (0.0008*dTms*dTms)//(mm/dt^2) 2m/s^2
#define MAX_SPEED (0.8*dTms)//mm/dt 時速3.6キロ(歩くスピード) 1m/s
#define START_SPEED 0
#define END_SPEED 0
#define MIDLE 10

#define RIGHT 1
#define LEFT 0

#define RAD_PULSE (2*PI/RESOLUTION)//rad/パルス数
#define MM_PULSE (RESOLUTION/(2.0*PI*RADIUS))//vからw'これをmm/dtに掛ければいい mm->pulse

enum {PIDCONTROL,SLOW,SLOWBACK,DAIKEI,STOP};


class MT{
    private:
    public:
        int Mode;
        int Direction;
        int T[3];
        int period;
        int count_time;

        int prcnt;
        int spd;
        int prev_errspd;
        int Dir;

        double kp;//0.0003//4000pulseぐらいでいいゲイン
        double ki;//0.000005
        double kd;

        double goal_mm;
        double restgoalspd_mm;
        double goalspd_mm;//mm

        int errspd;

        double acc;
        double dif;
        double dif_;


        double output_p;
        double output_v;
        double acc_output_v;
        double prev_output_v;
        double prev_output_p;

        int NAME;

        MT(int name);
        void MTSetGein(double p,double i,double d);
        void MTReset();
        double PID(int crr);
        void make_velo_plan(double mm);
};

#endif