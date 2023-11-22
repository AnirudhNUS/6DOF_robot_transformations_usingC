#include "ROBheaders.h"

int main ()
{
    string name;
    int i,j,num;
    num = 11;
    matrices base[6], transformed;  // 6 matricies for 6 transformations
    name = "matrix_source.txt";
    read_mat(base,name);

    name = "angle_source.txt";
    rob_kin* orig = new rob_kin[num];
    get_angles(orig,name,num);       

   // transformed = trans_mat(orig[2],base);

    matrices* trans = new matrices[num];

    mk_trans_arr(orig,base,trans,num);

    double** pos_res = new double*[num];
    double offset[4] = {0.1,0.3,0.2,1};

    make_pos(trans,pos_res,offset,num);
    
}
