#include"R2.hpp"

int main(){
    InitR2();
    int id[MOTIONSIZE] = {1,2,1,0,1,1,0};
    int arg[MOTIONSIZE] = {500,90,500,2000,2000,2000,2000,2000,2000,1000};
    R2MakeMotionList(1,id,arg);
    R2Simulation();

    return 0;
}
