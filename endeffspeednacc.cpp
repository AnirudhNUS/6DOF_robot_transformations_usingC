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

    double** xyz = new double*[3];   // make separate xyz
    double** fin = new double*[3];   // fin variable to get velocity
    double* tim = new double[num];    // make for time

    for(i=0;i<3;i++)
    {
        xyz[i] = new double[num];  // need to be fully declared
    }

    for(i=0;i<num;i++)
    {
        for(j=0;j<3;j++)
        {
            xyz[j][i] = pos_res[i][j];
        }
        tim[i] = orig[i].t;   // get time for origin
    }

    name = "end_ef_speed.txt";

    make_VoA_EE(xyz,tim,fin,name,num);

    double** acc = new double*[3];   // fin variable to get acceleration
    
    name = "end_ef_acc.txt";

    make_VoA_EE(fin,tim,acc,name,num);

}
