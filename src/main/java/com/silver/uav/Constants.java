package com.silver.uav;

public interface Constants {

    double CA = 180;
    int CR = 130;
    double CO = 200;
    int RAI = 10;
    int MASS = 1;
    int NUM_IN_GROUP = 2;

    double SP = 100;                //引领机间期望距离
    double U = 100;                 //参数
    double A = .13;                   //幅度参数
    double DENSITY = 3/(Math.PI*Math.pow(U/2, 2));

    int N_IN_GROUP = 3;
    int N_OF_GROUP = 13;

    double K_PX = 0.00001;
    double K_PY = 0.0001;
    double K_IX = 0.0001;
    double K_IY = 0.0001;
}
