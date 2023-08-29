#include"R2.hpp"
int main(){
    InitR2();
    sleep_for(2000ms);
    int id[MOTIONSIZE] = {0,1,0,1,1,0};
    int arg[MOTIONSIZE] = {500,500,500,500,500,2000,2000,2000,1000};
    R2MakeMotionList(3,id,arg);
    R2Simulation();
    return 0;
}
