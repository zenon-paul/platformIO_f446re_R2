#include"R2.hpp"
#define L 87
int main(){//87->90曲がる
    InitR2();
    sleep_for(2000ms);// .      w           0               w  
    int id[MOTIONSIZE] = {8,0    ,2, 0,  4, 1,   1, 8, 2, 1, 2, 5, 1,  3,   0,   1, 2 ,1, 2,4, 1, 2, 0, 5, 1, 3 ,4,7,8};
    int arg[MOTIONSIZE] = {2,1060,L,1795,0,300,1400,1,L,900,110,0,200,110-L,150,1200,L,900,L,0,360,L,800,0,800,L,0,1,1};
    R2MakeMotionList(1,id,arg);//7
    R2Simulation();
    return 0;
}
