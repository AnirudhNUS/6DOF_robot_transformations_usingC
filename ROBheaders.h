#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <iomanip>

using namespace std;

string line;        // read and write lines, name of reading files
fstream fileRead, fileWrite;    // Files to read and write

const double pi = 3.14159;     // for angle calculations

struct rob_kin   // robot kinematics
{
    double var[6], t;  
    // 6 variables for angle or speed or acceleration
    // each instance comes with its own timestamp t
};

void get_angles(rob_kin* arr, string source_name, int num)    
{   // Function to get the angles or kinematic varibles from a text file of filename "source_name"
    int i,j;

    fileRead.open(source_name, ios::in);
    cout<<"\nReading from file: "<<source_name<<"\n";
    
    getline(fileRead,line);    // heading line
    getline(fileRead,line);    // empty line

    for (i=0; i<num; i++)
    {
        cout<<"\nLine "<<(i+1)<<": \t";
        for (j=0; j<6; j++)
        {
            fileRead>>arr[i].var[j]; // array element  and variable
            cout<<arr[i].var[j]<<"\t";
        }
        fileRead>>arr[i].t;  // time stamp   
        cout<<"time: "<<arr[i].t;
    }

    fileRead.close(); 
    cout<<"\nThere were "<<i<<" configurations read \n";
}

void make_angles(rob_kin start, rob_kin end, string dest_name, int num)
{   // generate num angles between start and end points in a sinewave pattern
    rob_kin curr;
    double TD, cosbuf, dif;
    int i,j;

    fileWrite.open(dest_name, ios::out);   // truncate for the top fill
    cout<<"\nWriting on: "<<dest_name<<" file\nInitial 2 lines";

    for(i=0;i<6;i++)
    {  fileWrite<<"A"<<(i+1)<<"\t";    }

    fileWrite<<"T\n\n";  // time and then 2 lines off

    for(i=0;i<6;i++)
    {  fileWrite<<setprecision(3)<<start.var[i]<<"\t";   }
    fileWrite<<setprecision(3)<<start.t<<"\n";     // first angle same as start

    // angle = start + ((end-start)/2)( 1-cos(pi*(i/num)) )
    // time = start + (end - start)*(i/num)

    TD = end.t - start.t;

    for (i=1;i<num;i++)
    {
        for(j=0;j<6;j++)
        {
            cosbuf = cos((pi*i)/(num-1));
            dif = (end.var[j]-start.var[j])/2;
            curr.var[j] = start.var[j] + dif*(1-cosbuf);
            fileWrite<<setprecision(3)<<curr.var[j]<<"\t";
            cout<<setprecision(3)<<curr.var[j]<<"\t";
        }
        curr.t = start.t + (TD*i)/(num-1);
        fileWrite<<setprecision(3)<<curr.t<<"\n";
        cout<<"\nPrint line: "<<i<<"\t";
    }
    fileWrite.close();

}

void take_der(double* a, double* s, double* t, int num)
{       // take a derrivative. Convert angle/ position to speed, speed to acceleration
        
    cout<<"\nUsing take_der func \n";
    for (int i=1;i<(num-1);i++)
    {
        s[i] = (a[i+1]-a[i-1])/(t[i+1]-t[i-1]);
        cout<<"\nDerrivative value: "<<i<<" = "<<s[i];
    }

    s[0] = 2*s[1] - s[2];        // first value calc by s[1] being average of s[0] and s[2]
    s[num] = 2*s[num-1] - s[num-2];    // last value calculated by s[num-1] being avg of neighbours

}

void make_VoA(rob_kin* original, rob_kin* slope, int num, string sp_name)
{       // make velocity or acceleration from an original configuration by taking its derrivative
    cout<<"\nExecuting VoA function\n";
    int i,j;
    double* tim = new double[num]; // dynamically allocate array for time

    double** ang = new double*[6];
    for (i=0;i<6;i++)
    {   ang[i] = new double[num];    }    // angles dynamically allocated

    double** spe = new double*[6];
    for (i=0;i<6;i++)
    {   spe[i] = new double[num];    }    // speed dynamically allocated

    for (i=0;i<num;i++)
    {
        tim[i] = original[i].t;
        slope[i].t = original[i].t;  // put same time stamps in slope
        for(j=0;j<6;j++)
        {    ang[j][i] = original[i].var[j];  }
    }
    
    for(i=0;i<6;i++)
    {   take_der(ang[i],spe[i],tim,num);    }   // values converted and stored on spe

    // store in slope variable and write on file

    fileWrite.open(sp_name, ios::out);   // truncate for the top fill
    cout<<"\nWriting on: "<<sp_name<<" file\nInitial 2 lines";
    
    for(i=0;i<6;i++)
    {  fileWrite<<"P"<<(i+1)<<"\t";    }    // general P for velocity or acceleration

    fileWrite<<"T\n\n";  // time and then 2 lines off

    cout<<"\nStarting to write the derrivative values\n";

    for (i=0;i<num;i++)
    {
        for(j=0;j<6;j++)
        {    
            slope[i].var[j] = spe[j][i];
            if(spe[j][i]<0.001)
            {   
                slope[i].var[j] = 0;  // avoid very low values
                cout<<"\nValue too low, made zero\n";
            }       
            fileWrite<<setprecision(3)<<slope[i].var[j]<<"\t";
            cout<<setprecision(3)<<slope[i].var[j]<<"\t";
        }
        fileWrite<<setprecision(3)<<slope[i].t<<endl;
        cout<<setprecision(3)<<slope[i].t<<endl;
    }
    
    fileWrite.close(); 

    cout<<"\nFinished writing "<<i<<" lines\n";
}


////////////////// MATRICES SECTION ////////////////////

struct matrices
{
    double mat[4][4];
    double d[3];   // 3 d values remain constant for each joint
    char type;  // x, y or z rotation
};

void read_mat(matrices *foundation, string mat_source)
{   // foundation matrix array is to be referred to for type and d[3] values
    int i,j;
    fileRead.open(mat_source, ios::in);
    cout<<"\nReading from file: "<<mat_source<<"\n";
    
    getline(fileRead,line);    // heading line

    for(i=0;i<6;i++)
    {
        fileRead>>foundation[i].type;
        for(j=0;j<3;j++)
        {   fileRead>>foundation[i].d[j];     }
    }
    cout<<"\nFile read \n";
    fileRead.close();
}

matrices make_mat(double theta, matrices a)
{
    int i,j;
    for(i=0;i<4;i++)
    {
        for(j=0;j<4;j++)
        {  a.mat[i][j] = 0;   }  // set all values zero
    }

    for(i=0;i<3;i++)
    {   a.mat[i][3] = a.d[i];   }  // fill last column

    a.mat[3][3] = 1; // last place is 1

    if(a.type == 'x')
    {
        a.mat[0][0] = 1;
        a.mat[1][1] = cos(theta);
        a.mat[2][1] = sin(theta);
        a.mat[1][2] = -1*sin(theta);
        a.mat[2][2] = cos(theta);
    }

    else if(a.type == 'y')
    {
        a.mat[0][0] = cos(theta);
        a.mat[1][1] = 1;
        a.mat[0][2] = sin(theta);
        a.mat[2][0] = -1*sin(theta);
        a.mat[2][2] = cos(theta);
    }

    else if(a.type == 'z')
    {
        a.mat[0][0] = cos(theta);
        a.mat[1][1] = cos(theta);
        a.mat[1][0] = sin(theta);
        a.mat[0][1] = -1*sin(theta);
        a.mat[2][2] = 1;
    }

    else
    {   cout<<"\nInvalid type, please check file\n";    }

    // cout<<"\nPrint matrix\n";
    //  for(i=0;i<4;i++)
    // {
    //     for(j=0;j<4;j++)
    //     {  cout<<setprecision(3)<<a.mat[i][j]<<"\t";   }  

    //     cout<<endl;
    // }

    return a;
}

matrices mult_mat(matrices a, matrices b)
{
    matrices c;
    double sum;
    int i,j,k;

    for(i=0;i<4;i++)
    {
        for(j=0;j<4;j++)
        {
            sum = 0;
            for(k=0;k<4;k++)
            {
                sum += a.mat[i][k]*b.mat[k][j];
            }
            c.mat[i][j] = sum;
        }
    }
    
    // cout<<"\nPrint product matrix\n";
    //  for(i=0;i<4;i++)
    // {
    //     for(j=0;j<4;j++)
    //     {  cout<<setprecision(3)<<c.mat[i][j]<<"\t";   }  

    //     cout<<endl;
    // }

    return c;
}

matrices trans_mat(rob_kin ang,matrices *base)
{   // make transformation matrix from set of 6 angles and the foundation matrix
    // base array gives the values of d[] and type
    int i,j;
    matrices result, temp, temp1;
    matrices buf[6];

    for (i=0;i<6;i++)
    {
        buf[i] = make_mat(ang.var[i],base[i]);
        // all the 6 matrices for each joint
    }
    // multiply together to get result
    temp = buf[0];
    for(i=1;i<6;i++)
    {
        result = mult_mat(temp,buf[i]);
        temp = result;
    }
    
    return result;
}

void mk_trans_arr(rob_kin *ang, matrices *base, matrices *trans, int num)
{   // makes transformation matrix for every timestep
    // store these transformation matrices in a dedicated file
    int i,j,k;
    string name;
    for(i=0;i<num;i++)
    {
        trans[i] = trans_mat(ang[i],base);
    }
    cout<<"\nFilling the transformation file now\n";

    fileWrite.open("trans_mats.txt", ios::out);

    for(i=0;i<num;i++)
    {
        cout<<"\nMatrix no: "<<i<<endl;
        fileWrite<<"\n\n\nMatrix no: "<<i<<"\n\n";

        for(j=0;j<4;j++)
        {
            for(k=0;k<4;k++)
            {
                fileWrite<<setprecision(3)<<trans[i].mat[j][k]<<"\t";
            }
            fileWrite<<"\n";
        }
    }

    fileWrite.close();
}

void make_pos(matrices *trans, double **pos_res, double *offset, int num)
{       // take a fixed position wrt the end effector and get position by angles
    // store the transformed resultant positions in a dedicated file
    int i,j,k;
    double sum;

    for(i=0;i<num;i++)
    {
        pos_res[i] = new double[4];
    }

    for(i=0;i<num;i++)
    {
        for(j=0;j<3;j++)
        {
            sum = 0;
            for(k=0;k<4;k++)
            {
                sum += trans[i].mat[j][k]*offset[k];
            }
            pos_res[i][j] = sum;
        }
        pos_res[i][3] = 1;
    }

    fileWrite.open("end_effector_positions.txt", ios::out);

    for(i=0;i<num;i++)
    {
        for(j=0;j<4;j++)
        {
            fileWrite<<setprecision(3)<<pos_res[i][j]<<"\t";
        }
        fileWrite<<"\n";
    }

    fileWrite.close();

}

void make_VoA_EE(double** xyz, double* tim, double **fin, string sp_name, int num)
{       // make velocity or acceleration from an xyz position map by taking its derrivative
    cout<<"\nExecuting VoA EE function\n";
    int i,j,k;
    for (i=0;i<3;i++)
    {   
        fin[i] = new double[num];  
    }

    for(i=0;i<3;i++)
    {    
        take_der(xyz[i],fin[i],tim,num);
    }

    fileWrite.open(sp_name, ios::out);
    fileWrite<<"X \t Y \t Z \t T\n\n";
    cout<<"\nWriting on file\n";

    for(i=0;i<num;i++)
    {
        for(j=0;j<3;j++)
        {
            fileWrite<<setprecision(3)<<fin[j][i]<<"\t";
            cout<<setprecision(3)<<fin[j][i]<<"\t";
        }
        fileWrite<<tim[i]<<endl;
        cout<<tim[i]<<endl;
    }

    fileWrite.close();
    
    cout<<"\nDone writing file\n";

}