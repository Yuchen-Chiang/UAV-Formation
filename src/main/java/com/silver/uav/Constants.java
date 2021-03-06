package com.silver.uav;

public interface Constants {

    double CA = 0.15;
    double CR = 1;
    double CO = 0.2;
    int RAI = 10;
    int MASS = 1;
    int NUM_IN_GROUP = 2;

    double SP = 100;                //引领机间期望距离
    double U = 100;                 //参数
    double AA = .13;                   //幅度参数
    double AR = 0.3;
    double DENSITY = 3/(Math.PI*Math.pow(U/2, 2));

    int N_IN_GROUP = 3;
    int N_OF_GROUP = 6;

    double K_PX = 0.00001;
    double K_PY = 0.0001;
    double K_IX = 0.0001;
    double K_IY = 0.0001;

    double K_PX_O = 0.3;
    double K_PY_O = 0.000001;
    double K_IX_O = 0.001;
    double K_IY_O = 0.000001;
}
