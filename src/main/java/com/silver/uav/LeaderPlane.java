package com.silver.uav;

public class LeaderPlane extends Plane {

    Double density;

    public LeaderPlane(Double x,
                       Double y,
                       Double vx,
                       Double vy,
                       Double density,
                       int id) {
        super(x, y, vx, vy, id);
        this.density = density;
    }

}
