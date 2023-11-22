#include "ROBheaders.h"

int main ()
{
    string name;
    int num;
    rob_kin* orig = new rob_kin[num];
    rob_kin* fin = new rob_kin[num];

    num = 11;                    // 11 points between start and end
    name = "speed_vals.txt";       // put info in this new text file for speed values
    //make_VoA(orig,fin,num,name);

    get_angles(orig,name,num);       // new original file for speed to make acceleration
    //orig[0].var[1] = 0;

    name = "acc_vals.txt";       // put info in this new text file for acceleration values
    make_VoA(orig,fin,num,name);
    cout<<"\nTried makeVoA\n";
}
