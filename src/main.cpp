#include"R2.hpp"
int main(){
    InitR2();
    sleep_for(2000ms);
    int id[MOTIONSIZE] = {0,3,0,2,4,1,2,1,2,0};
    int arg[MOTIONSIZE] = {1650,83,900,83,0,370,83,900,83,1100};
    R2MakeMotionList(3,id,arg);
    R2Simulation();
    return 0;
}
