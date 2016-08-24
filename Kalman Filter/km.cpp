#include <fstream>
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
//my matrix library, you can use your own favorite matrix library
#include "matrix.h"
#define TIMESTEP 0.05

using namespace std;

float timeIndex,m_a1,m_a2,m_a3,c_a1,c_a2,c_a3,tau1,tau2,tau3;
float a1,a2,a3,a1d,a2d,a3d,a1dd,a2dd,a3dd;
float new_a1d, new_a2d, new_a3d;
float TIME=0;
matrix A(6,6);
matrix B(6,3);
matrix C=matrix::eye(6);  //initialize them as 6×6 identity matrices
matrix Q=matrix::eye(6);
matrix R=matrix::eye(6);
matrix y(6,1);
matrix K(6,6);
matrix x(6,1);
matrix state(6,1);
matrix action(3,1);
matrix lastState(6,1);
matrix P=matrix::eye(6);
matrix p=matrix::eye(6);
matrix measurement(6,1);

void initKalman(){

float a[6][6]={
{1.004,    0.0001,  0.001,    0.0014,    0.0000,  -0.0003  },
{0.000,    1.000,     -0.00,      0.0000,    0.0019,   0          },
{0.0004,  0.0002,  1.002,    0.0003,    0.0001,   0.0015   },
{0.2028,  0.0481,  0.0433,  0.7114,   -0.0166,  -0.1458   },
{0.0080,  0.0021, -0.0020, -0.0224,    0.9289,   0.005    },
{0.1895,  0.1009,  0.1101, -0.1602,    0.0621,   0.7404  }
};

float b[6][3] = {
{0.0000,   0.0000 ,  0.0000  },
{0.0000,   0.0000,  -0.0000  },
{0.0000,  -0.0000,   0.0000  },
{0.007,    -0.0000,   0.0005 },
{0.0001,   0.0000,  -0.0000 },
{0.0003,  -0.0000,   0.0008 }
};

/*loads the A and B matrices from above*/
for (int i = 0; i < 6; i++){
for (int j = 0; j < 6; j++){
A[i][j]=a[i][j];
}
}
for (int i = 0; i < 6; i++){
for (int j = 0; j < 3; j++){
B[i][j]=b[i][j];
}
}

/*initializes the state*/
state[0][0]=0.1;
state[1][0]=0.1;
state[2][0]=0.1;
state[3][0]=0.1;
state[4][0]=0.1;
state[5][0]=0.1;

lastState=state;

}

void kalman(){
lastState=state;
state[0][0]=c_a1;
state[1][0]=c_a2;
state[2][0]=c_a3;
state[3][0]=a1d;
state[4][0]=a2d;
state[5][0]=a3d;

measurement[0][0]=m_a1;
measurement[1][0]=m_a2;
measurement[2][0]=m_a3;
measurement[3][0]=a1d;
measurement[4][0]=a2d;
measurement[5][0]=a3d;

action[0][0]=tau1;
action[1][0]=tau2;
action[2][0]=tau3;

matrix temp1(6,6);
matrix temp2(6,6);
matrix temp3(6,6);
matrix temp4(6,1);
/************ Prediction Equations*****************/
x = A*lastState + B*action;
p = A*P*A’ + Q;
/************ Update Equations **********/
K = p*C*pinv(C*p*C’+R);

y=C*state;

state = x + K*(y-C*lastState);

P = (eye(6) – K*C)*p;

a1=state[0][0];
a2=state[1][0];
a3=state[2][0];
a1d=state[3][0];
a2d=state[4][0];
a3d=state[5][0];
}

/* This function is not used since I am using position to get velocity (i.e. differentiation).
* However I think that it is useful to include if you want velocity and position
* from acceleration you would use it */
void integrate(){
new_a1d = a1d + a1dd*TIMESTEP;
a1 += (new_a1d + a1d)*TIMESTEP/2;
a1d = new_a1d;
new_a2d = a2d + a2dd*TIMESTEP;
a2 += (new_a2d + a2d)*TIMESTEP/2;
a2d = new_a2d;
new_a3d = a3d + a3dd*TIMESTEP;
a3 += (new_a3d + a3d)*TIMESTEP/2;
a3d = new_a3d;
TIME+=TIMESTEP;
}

/*This gives me velocity from position*/
void differentiation(){
a1d=(state[0][0]-lastState[0][0])/TIMESTEP;
a2d=(state[1][0]-lastState[1][0])/TIMESTEP;
a3d=(state[2][0]-lastState[2][0])/TIMESTEP;
TIME+=TIMESTEP;
}

int main () {
initKalman();
char buffer[500];
ifstream readFile (“DATA.txt”); // this is where I read my data since I am proccessing it all offline

while (!readFile.eof()){
readFile.getline (buffer,500);
sscanf(buffer, “%f %f %f %f %f %f %f %f %f %f “,&timeIndex,&m_a1,&m_a2,&m_a3,&c_a1,&c_a2,&c_a3,&tau1,&tau2,&tau3);

kalman();
differentiation();
//integrate();

/*this is where I log the results to and/or print it to the screen */
FILE *file=fopen(“filterOutput.txt”, “a”);
fprintf(file,”%f %f %f %f %f %f %f %f %f %f \n”,TIME,a1,a2,a3,a1d,a2d,a3d,tau1,tau2,tau3);
fprintf(stderr,”%f %f %f %f %f %f %f %f %f %f \n”,TIME,a1,a2,a3,a1d,a2d,a3d,tau1,tau2,tau3);
fclose(file);
}
return 1;
}

