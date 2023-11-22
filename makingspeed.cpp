#include "ROBheaders.h"

int main ()
{
    string name;
    int num = 2;      // for start and end points
    rob_kin st, en;
    name = "strtNend.txt";  // start and end positions
    rob_kin* anglesArr = new rob_kin[num]; // dynamically allocate array
    get_angles(anglesArr,name,num);

    st = anglesArr[0];
    en = anglesArr[1];
    num = 11;                    // 11 points between start and end
    name = "angle_source.txt";
    make_angles(st, en, name, num);

    rob_kin* orig = new rob_kin[num];
    rob_kin* fin = new rob_kin[num];
    get_angles(orig,name,num);       // keep the name angle_source for now
    // store the values in orig variable
    // this is how we extract info from a text file to simulate the API to and fro between ROS and C++
    
    name = "speed_vals.txt";       // put info in this new text file
    make_VoA(orig,fin,num,name);
    
}