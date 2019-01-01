package com.silver.uav;

public class LeaderPlane extends Plane {

    double density;
    double v;
    double theta;

    public LeaderPlane(Double x,
                       Double y,
                       Double vx,
                       Double vy,
                       double density,
                       int id) {
        super(x, y, vx, vy, id);
        this.density = density;
    }

    public LeaderPlane(double x,
                       double y,
                       double v,
                       double theta,
                       double density,
                       int id) {
        super(x, y, .0, .0, id);
        this.density = density;
        this.v = v;
        this.theta = theta;
    }

}
