package com.silver.uav;

abstract class Plane {

    Double x;
    Double y;

    Double vx;
    Double vy;

    Double ax;
    Double ay;

    Double x1;
    Double y1;

    Double vx1;
    Double vy1;

    Double ax1;
    Double ay1;

    int id;

    Plane(Double x, Double y, Double vx, Double vy, int id) {
        this.x = x;
        this.y = y;
        this.vx = vx;
        this.vy = vy;
        this.ax = .0;
        this.ay = .0;

        this.x1 = x;
        this.y1 = y;
        this.vx1 = vx;
        this.vy1 = vy;
        this.ax1 = ax;
        this.ay1 = ay;

        this.id = id;
    }
}
