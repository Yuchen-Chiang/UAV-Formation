package com.silver.uav;

import java.util.List;

public class InnerController {

    private static int LA = 3*Constants.RAI;
    private static int LO = 3*Constants.RAI;
    private static int LR1 = 2*Constants.RAI;
    private static double LR2 = Math.sqrt(2*Math.pow((double)Constants.RAI, 2)*(1-Math.cos(2*Math.PI/Constants.N_IN_GROUP)));

    static void printCurrentPlane(double simulationTime, LeaderPlane lp, List<FollowerPlane> fps) {

        System.out.println("simulation time =" + simulationTime);
        System.out.println("leader plane (" + lp.x + ", " + lp.y + ")");
        for (FollowerPlane fp : fps) System.out.println("follower plane " + fp.id + " (" + fp.x + ", " + fp.y + ")");
        System.out.println("--------------------------------------");
    }

    static double[] computeAcceleration(LeaderPlane lp, FollowerPlane fp, List<FollowerPlane> fps) {

        double[] a = {.0, .0};
        if (computeDistance(lp, fp) <= LA) {
            double[] tmp = computeAttract(lp, fp);
            a[0] += tmp[0];
            a[1] += tmp[1];
        }
        if (computeDistance(lp, fp) <= LR1) {
            double[] tmp = computeRepel1(lp, fp);
            a[0] += tmp[0];
            a[1] += tmp[1];
        }
        if (computeDistance(lp, fp) <= LO  && computeVelocity(lp, fp) >=0.0001) {
            double[] tmp = computeOrigin(lp, fp);
            a[0] += tmp[0];
            a[1] += tmp[1];
        }
        for (FollowerPlane p : fps) {
            if (p.id != fp.id && computeDistance(fp, p) <= LR2) {
                double[] tmp = computeRepel2(fp, p);
                a[0] += tmp[0];
                a[1] += tmp[1];
            }
        }

        return a;
    }

    static double[] computeAttract(LeaderPlane lp, FollowerPlane fp) {

        double distance = computeDistance(lp, fp);
        double force = (Constants.CA/(double)LA)*Math.exp(-1*distance/(double)LA);
        double sin = (lp.y-fp.y)/distance;
        double cos = (lp.x-fp.x)/distance;
        double a0 = force/Constants.MASS;
        double[] a = new double[2];
        a[0] = a0*cos;
        a[1] = a0*sin;
        return a;
    }

    static double[] computeRepel1(LeaderPlane lp, FollowerPlane fp) {

        double distance = computeDistance(lp, fp);
        double force = (Constants.CR/(double)LR1)*Math.exp(-1*distance/(double)LR1);
        double cos = (fp.x-lp.x)/distance;
        double sin = (fp.y-lp.y)/distance;
        double a = force/Constants.MASS;
        return new double[]{a*cos, a*sin};
    }

    static double[] computeRepel2(FollowerPlane tfp, FollowerPlane fp) {

        double distance = computeDistance(tfp, fp);
        double force = (Constants.CR/LR2)*Math.exp(-1*distance/LR2);
        double sin = (tfp.y-fp.y)/distance;
        double cos = (tfp.x-fp.x)/distance;
        double a = force/Constants.MASS;
        return new double[]{a*cos, a*sin};
    }

    static double[] computeOrigin(LeaderPlane lp, FollowerPlane fp) {

        double distance = computeDistance(lp, fp);
        double force = (Constants.CO/(double)LO)*Math.exp(-1*distance/(double)LO);
        double velocity = computeVelocity(lp, fp);
        double cos = (lp.vx-fp.vx)/velocity;
        double sin = (lp.vy-fp.vy)/velocity;
        double a = force/Constants.MASS;
        return new double[]{a*cos, a*sin};
    }

    private static double computeDistance(Plane p1, Plane p2) {
        return Math.sqrt(Math.pow(p1.x-p2.x, 2)+Math.pow(p1.y-p2.y, 2));
    }

    private static double computeVelocity(Plane p1, Plane p2) {
        return Math.sqrt(Math.pow(p1.vx-p2.vx, 2)+Math.pow(p1.vy-p2.vy, 2));
    }
}
