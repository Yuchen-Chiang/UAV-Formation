package com.silver.uav;

import ProGAL.geom2d.Point;
import ProGAL.geom2d.delaunay.Triangle;

import java.io.IOException;
import java.io.RandomAccessFile;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

class OuterController {

    static void initMemory(Map<LeaderPlane, Set<LeaderPlane>> memory, List<LeaderPlane> groups) {
        for (int i = 1; i < Constants.N_OF_GROUP; i++) {
            switch (i) {
                case 1 :
                    Set<LeaderPlane> set1 = new HashSet<>();
                    set1.add(groups.get(0));
                    set1.add(groups.get(2));
                    memory.put(groups.get(1), set1);
                    break;
                case 2 :
                    Set<LeaderPlane> set2 = new HashSet<>();
                    set2.add(groups.get(0));
                    set2.add(groups.get(1));
                    memory.put(groups.get(2), set2);
                    break;
                case 3 :
                    Set<LeaderPlane> set3 = new HashSet<>();
                    set3.add(groups.get(1));
                    set3.add(groups.get(4));
                    memory.put(groups.get(3), set3);
                    break;
                case 4 :
                    Set<LeaderPlane> set4 = new HashSet<>();
                    set4.add(groups.get(3));
                    set4.add(groups.get(1));
                    set4.add(groups.get(2));
                    set4.add(groups.get(5));
                    memory.put(groups.get(4), set4);
                    break;
                case 5 :
                    Set<LeaderPlane> set5 = new HashSet<>();
                    set5.add(groups.get(4));
                    set5.add(groups.get(2));
                    memory.put(groups.get(5), set5);
                    break;
            }
        }
    }

    static void printDistance(List<LeaderPlane> lps) {

        LeaderPlane p0 = lps.get(0);
        LeaderPlane p1 = lps.get(1);
        LeaderPlane p2 = lps.get(2);
        LeaderPlane p3 = lps.get(3);
        LeaderPlane p4 = lps.get(4);

        double dist01 = distance(p0, p1);
        double dist12 = distance(p1, p2);
        double dist13 = distance(p1, p3);
        double dist04 = distance(p0, p4);

        try (RandomAccessFile file1 = new RandomAccessFile("C:\\Users\\Yuche\\Documents\\MATLAB\\outer\\outer_dist_01", "rw");
             RandomAccessFile file4 = new RandomAccessFile("C:\\Users\\Yuche\\Documents\\MATLAB\\outer\\outer_dist_04", "rw");
             RandomAccessFile file3 = new RandomAccessFile("C:\\Users\\Yuche\\Documents\\MATLAB\\outer\\outer_dist_12", "rw");
             RandomAccessFile file5 = new RandomAccessFile("C:\\Users\\Yuche\\Documents\\MATLAB\\outer\\outer_dist_13", "rw")) {
            long l1 = file1.length();
            file1.seek(l1);
            file1.writeBytes(dist01+" ");

            long l4 = file4.length();
            file4.seek(l4);
            file4.writeBytes(dist04+" ");

            long l3 = file3.length();
            file3.seek(l3);
            file3.writeBytes(dist12+" ");

            long l5 = file5.length();
            file5.seek(l5);
            file5.writeBytes(dist13+" ");
        } catch (IOException ioe) {
            ioe.printStackTrace();
        }

    }

    static void printToText(List<LeaderPlane> lps) {

        for (int i = 0; i < lps.size(); i++) {
            Plane p = lps.get(i);
            try (RandomAccessFile fileX = new RandomAccessFile("C:\\Users\\Yuche\\Documents\\MATLAB\\outer\\outer_"+i+"_x", "rw");
                 RandomAccessFile fileY = new RandomAccessFile("C:\\Users\\Yuche\\Documents\\MATLAB\\outer\\outer_"+i+"_y", "rw")) {
                long lX = fileX.length();
                fileX.seek(lX);
                fileX.writeBytes(p.x+" ");

                long lY = fileY.length();
                fileY.seek(lY);
                fileY.writeBytes(p.y+" ");
            } catch (IOException ioe) {
                ioe.printStackTrace();
            }
        }
    }

    static void printCurrentPlane(double simulationTime, List<LeaderPlane> pll) {

        System.out.println("simulation time =" + simulationTime);
        for (LeaderPlane lp : pll) {
            System.out.println("leader plane " + lp.id + " (" + lp.x + ", " + lp.y + ")");
        }
        System.out.println("--------------------------------------");

    }

    static void processOuter2_1(LeaderPlane lp, double f) {

        lp.x = lp.x1;
        lp.y = lp.y1;
        lp.vx = lp.vx1;
        lp.vy = lp.vy1;

        lp.x1 += lp.vx*f;
        lp.y1 += lp.vy*f;
    }

    static void processOuter2(LeaderPlane lp, Set<LeaderPlane> set, double f) {

        double[] v = {.0, .0};

        double[] vc = {.0, .0};
        for (LeaderPlane p : set) {
            double[] tmp = computeVelocityChange(lp, p);
            vc[0] += tmp[0];
            vc[1] += tmp[1];
        }

        double[] vw = {.0, .0};
        for (LeaderPlane p : set) {
            if (p.id/10 < lp.id/10) {
                double[] tmp = computeVelocityWithLeader(p, lp);
                vw[0] += tmp[0];
                vw[1] += tmp[1];
            }
        }

        double k1 = 0.2;
        double k2 = .009;
        v[0] = k1*vc[0] + k2*vw[0];
        v[1] = k1*vc[1] + k2*vw[1];

        lp.x = lp.x1;
        lp.y = lp.y1;
        lp.vx = lp.vx1;
        lp.vy = lp.vy1;

        lp.x1 += lp.vx*f;
        lp.y1 += lp.vy*f;
        lp.vx1 = lp.vx1 + v[0];
        lp.vy1 = lp.vy1 + v[1];
    }

    private static double[] computeVelocityChange(LeaderPlane lp, LeaderPlane p) {

        double s = distance(lp, p);
        double f;
        double[] v = new double[]{.0,.0};
        if (s > Constants.SP && s < 2*Constants.SP) {
            f = Constants.AA*Math.abs(Math.log(1+Constants.U*Math.abs((s-Constants.SP)/Constants.SP))/Math.log(1+Constants.U));
            double cos = (p.x-lp.x)/s;
            double sin = (p.y-lp.y)/s;
            v[0] = f*cos;
            v[1] = f*sin;
        } else if (s > 0 && s < Constants.SP){
            f = Constants.AR*Math.abs(Math.log(1+Constants.U*Math.abs((s-Constants.SP)/Constants.SP))/Math.log(1+Constants.U));
            double cos = (lp.x-p.x)/s;
            double sin = (lp.y-p.y)/s;
            v[0] = f*cos;
            v[1] = f*sin;
        }

        return v;
    }

    private static double[] computeVelocityWithLeader(LeaderPlane lp, LeaderPlane p) {

        double v = velocity(lp, p);
        if (v < 0.000000001) return new double[]{.0, .0};
        double cos = (lp.vx-p.vx)/v;
        double sin = (lp.vy-p.vy)/v;
        return new double[]{lp.vx-p.vx, lp.vy-p.vy};
    }

    static void processOuter(LeaderPlane lp, Set<LeaderPlane> set , double f) {

        double[] v = {.0, .0};

        Set<Point> sp = new HashSet<>();
        Point tp = new Point(lp.x, lp.y);
//        for (Triangle t : tl) {
//            checkAndAdd(t, tp, sp);
//        }

//        if (checkDensity(tp, sp)) {
//            for (LeaderPlane leader : set) {
//                Point p = new Point(leader.x, leader.y);
//                double[] tmp = computeExpectedVelocity(tp, p);
//                v[0] += tmp[0];
//                v[1] += tmp[1];
//            }
//        }

        for (LeaderPlane leader : set) {
            Point p = new Point(leader.x, leader.y);
            double[] tmp = computeExpectedVelocity(tp, p);
            v[0] += tmp[0];
            v[1] += tmp[1];
        }

        double alpha = .5;

        lp.x = lp.x1;
        lp.y = lp.y1;
        lp.vx = lp.vx1;
        lp.vy = lp.vy1;

        lp.x1 += lp.vx*f;
        lp.y1 += lp.vy*f;
        lp.vx1 = alpha*lp.vx1 + (1-alpha)*(lp.vx1 + v[0]);
        lp.vy1 = alpha*lp.vy1 + (1-alpha)*(lp.vy1 + v[1]);
    }

    private static boolean checkDensity(Point tp, Set<Point> sp) {

        double dist = 0;
        int n = sp.size();
        for (Point p : sp) {
            dist += distance(tp, p)/n;
        }
        double density = (Constants.NUM_IN_GROUP+1)/(Math.PI*Math.pow(dist, 2));
        return Math.abs(density-Constants.DENSITY)/Constants.DENSITY > 0.1;
    }

    private static double[] computeExpectedVelocity(Point tp, Point p) {

        double s = distance(tp, p);
        double f;
        double[] v = new double[]{.0,.0};
        if (s > 2*Constants.SP && s < 3*Constants.SP) {
            f = Constants.AA*Math.abs(Math.log(1+Constants.U*Math.abs((s-3*Constants.SP)/Constants.SP))/Math.log(1+Constants.U));
            double cos = (p.x()-tp.x())/s;
            double sin = (p.y()-tp.y())/s;
            v[0] = f*cos;
            v[1] = f*sin;
        } else
        if (s > Constants.SP && s < 2*Constants.SP) {
            f = Constants.AA*Math.abs(Math.log(1+Constants.U*Math.abs((s-Constants.SP)/Constants.SP))/Math.log(1+Constants.U));
            double cos = (p.x()-tp.x())/s;
            double sin = (p.y()-tp.y())/s;
            v[0] = f*cos;
            v[1] = f*sin;
        } else if (s > 0 && s < Constants.SP){
            f = Constants.AA*Math.abs(Math.log(1+Constants.U*Math.abs((s-Constants.SP)/Constants.SP))/Math.log(1+Constants.U));
            double cos = (tp.x()-p.x())/s;
            double sin = (tp.y()-p.y())/s;
            v[0] = f*cos;
            v[1] = f*sin;
        }

        return v;
    }

    static boolean testAndCheck(List<Triangle> ts, LeaderPlane t, LeaderPlane p) {

        Point tp = new Point(t.x, t.y);
        Point pp = new Point(p.x, p.y);
        for (Triangle tr : ts) {
            if (tp.equals(tr.getCorner(0))) {
                if (pp.equals(tr.getCorner(1)) || pp.equals(tr.getCorner(2))) return true;
            }
            else if (tp.equals(tr.getCorner(1))) {
                if (pp.equals(tr.getCorner(0)) || pp.equals(tr.getCorner(2))) return true;
            }
            else if (tp.equals(tr.getCorner(2))) {
                if (pp.equals(tr.getCorner(1)) || pp.equals(tr.getCorner(0))) return true;
            }
        }
        return false;
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

    private static double distance(Plane p1, Plane p2) {
        return Math.sqrt(Math.pow(p1.x-p2.x, 2)+Math.pow(p1.y-p2.y, 2));
    }

    private static double velocity(Plane p1, Plane p2) {
        return Math.sqrt(Math.pow(p1.vx-p2.vx, 2)+Math.pow(p1.vy-p2.vy, 2));
    }
}
