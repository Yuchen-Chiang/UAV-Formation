package com.silver.uav;

import ProGAL.geom2d.Point;
import ProGAL.geom2d.delaunay.Triangle;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

class OuterController {

    static void printCurrentPlane(double simulationTime, List<List<Plane>> pll) {

        System.out.println("simulation time =" + simulationTime);
        for (List<Plane> pl : pll) {
            for (int i = 0; i <= Constants.N_IN_GROUP; i++) {
                if (i == 0) System.out.println("leader plane " + pl.get(0).id + " (" + pl.get(0).x + ", " + pl.get(0).y + ")");
                else System.out.println("follower plane " + pl.get(i).id + " (" + pl.get(i).x + ", " + pl.get(i).y + ")");
            }
            System.out.println("--------------------------------------");
        }
    }

    static void processOuter(LeaderPlane lp, List<Triangle> tl, double f) {

        if (lp.id == Constants.LEADER_ID) {
            lp.x = lp.x1;
            lp.y = lp.y1;
            lp.x1 += f*lp.vx;
            lp.y1 += f*lp.vy;
        } else {

            double[] a = {.0, .0};

            Set<Point> sp = new HashSet<>();
            Point tp = new Point(lp.x, lp.y);
            for (Triangle t : tl) {
                checkAndAdd(t, tp, sp);
            }

            for (Point p : sp) {
                double[] tmp = computeAcceleration(tp, p);
                a[0] += tmp[0];
                a[1] += tmp[1];
            }

            lp.x = lp.x1;
            lp.y = lp.y1;
            lp.vx = lp.vx1;
            lp.vy = lp.vy1;
            lp.ax = lp.ax1;
            lp.ay = lp.ay1;

            lp.x1 += lp.vx*f + 0.5*lp.ax*f*f;
            lp.y1 += lp.vy*f + 0.5*lp.ay*f*f;
            lp.vx1 += lp.ax*f;
            lp.vy1 += lp.ay*f;
            lp.ax1 = a[0];
            lp.ay1 = a[1];
        }
    }

    private static double[] computeAcceleration(Point tp, Point p) {
        double dist = distance(tp, p);
        double param = 0.5*dist/Constants.EXPECT_R-1+Constants.MI/Constants.LI;
        double a = Constants.MI/Math.pow(param, 2)-Constants.LI/param;
        if (dist < 2*Constants.EXPECT_R) {
            double cos = (p.x()-tp.x())/dist;
            double sin = (p.y()-tp.y())/dist;
            return new double[]{a*cos, a*sin};
        } else if (dist >= 2*Constants.EXPECT_R && dist < Constants.COMMUNICATION_D) {
            double cos = (tp.x()-p.x())/dist;
            double sin = (tp.y()-p.y())/dist;
            return new double[]{a*cos, a*sin};
        } else {
            return new double[]{.0, .0};
        }
    }

    private static void checkAndAdd(Triangle t, Point p, Set<Point> sp) {
        if (p.equals(t.getCorner(0))) {
            sp.add(t.getCorner(1));
            sp.add(t.getCorner(2));
        }
        else if (p.equals(t.getCorner(1))) {
            sp.add(t.getCorner(0));
            sp.add(t.getCorner(2));
        }
        else if (p.equals(t.getCorner(2))) {
            sp.add(t.getCorner(0));
            sp.add(t.getCorner(1));
        }
    }

    private static double distance(Point tp, Point p) {
        return Math.sqrt(Math.pow(tp.x()-p.x(), 2)+Math.pow(tp.y()-p.y(), 2));
    }
}
