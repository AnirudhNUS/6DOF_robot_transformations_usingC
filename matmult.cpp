#include "ROBheaders.h"

int main ()
{
    string name;
    int i,j,num;
    num = 11;
    matrices base[6];  // 6 matricies for 6 transformations
    name = "matrix_source.txt";
    read_mat(base,name);

    matrices p,q,r;
    p = make_mat(0.32,base[1]);
    q = make_mat(0.98,base[4]);

    r = mult_mat(p,q);
    
    cout<<"\nPrint product matrix\n";
     for(i=0;i<4;i++)
    {
        for(j=0;j<4;j++)
        {  cout<<setprecision(3)<<r.mat[i][j]<<"\t";   }  

        cout<<endl;
    }
    
}
