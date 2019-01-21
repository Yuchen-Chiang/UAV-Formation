package com.silver.uav;

import java.io.IOException;
import java.io.RandomAccessFile;
import java.util.ArrayList;
import java.util.List;

public class InnerController {

    private static int LA = 5*Constants.RAI;
    private static int LO = 5*Constants.RAI;
    private static double LR1 = 2*Constants.RAI;
    private static double LR2 = Constants.RAI;

    static void computeXYError(LeaderPlane p0, List<FollowerPlane> fps) {

        FollowerPlane p1 = fps.get(0);
        FollowerPlane p2 = fps.get(1);

        double dist01 = Math.sqrt(Math.pow(p1.x-p0.x, 2)+Math.pow(p1.y-p0.y, 2));
        double dist02 = Math.sqrt(Math.pow(p2.x-p0.x, 2)+Math.pow(p2.y-p0.y, 2));
        double dist12 = Math.sqrt(Math.pow(p2.x-p1.x, 2)+Math.pow(p2.y-p1.y, 2));

        try (RandomAccessFile file1 = new RandomAccessFile("C:\\Users\\Yuche\\Documents\\MATLAB\\inner\\in_dist_01", "rw");
             RandomAccessFile file2 = new RandomAccessFile("C:\\Users\\Yuche\\Documents\\MATLAB\\inner\\in_dist_02", "rw");
             RandomAccessFile file3 = new RandomAccessFile("C:\\Users\\Yuche\\Documents\\MATLAB\\inner\\in_dist_12", "rw")) {
            long l1 = file1.length();
            file1.seek(l1);
            file1.writeBytes(dist01+" ");

            long l2 = file2.length();
            file2.seek(l2);
            file2.writeBytes(dist02+" ");

            long l3 = file3.length();
            file3.seek(l3);
            file3.writeBytes(dist12+" ");
        } catch (IOException ioe) {
            ioe.printStackTrace();
        }

    }

    static void printToText(LeaderPlane lp, List<FollowerPlane> pl) {

        try (RandomAccessFile fileX = new RandomAccessFile("C:\\Users\\Yuche\\Documents\\MATLAB\\inner\\in_"+0+"_x", "rw");
             RandomAccessFile fileY = new RandomAccessFile("C:\\Users\\Yuche\\Documents\\MATLAB\\inner\\in_"+0+"_y", "rw")) {
            long lX = fileX.length();
            fileX.seek(lX);
            fileX.writeBytes(lp.x+" ");

            long lY = fileY.length();
            fileY.seek(lY);
            fileY.writeBytes(lp.y+" ");
        } catch (IOException ioe) {
            ioe.printStackTrace();
        }

        for (int i = 1; i <= pl.size(); i++) {
            Plane p = pl.get(i-1);
            try (RandomAccessFile fileX = new RandomAccessFile("C:\\Users\\Yuche\\Documents\\MATLAB\\inner\\in_"+i+"_x", "rw");
                 RandomAccessFile fileY = new RandomAccessFile("C:\\Users\\Yuche\\Documents\\MATLAB\\inner\\in_"+i+"_y", "rw")) {
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

    static void printCurrentPlane(double simulationTime, LeaderPlane lp, List<FollowerPlane> fps) {

        System.out.println("simulation time =" + simulationTime);
        System.out.println("leader plane (" + lp.x + ", " + lp.y + ")");
        for (FollowerPlane fp : fps) System.out.println("follower plane " + fp.id + " (" + fp.x + ", " + fp.y + ")");
        System.out.println("--------------------------------------");
    }

    static double[] computeAcceleration2(int i, LeaderPlane lp, FollowerPlane fp, List<FollowerPlane> fps) {

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
        for (FollowerPlane p : fps) {
            if (p.id != fp.id && computeDistance(fp, p) <= LR2) {
                double[] tmp = computeRepel2(fp, p);
                a[0] += tmp[0];
                a[1] += tmp[1];
            }
        }

        if (computeDistance(lp, fp) <= LO) {
            if (i > 3000) {
                double[] tmp = computeOrigin(lp, fp);
                a[0] += tmp[0];
                a[1] += tmp[1];
            } else {
                double[] tmp = computeOrigin(lp, fp);
                a[0] += tmp[0];
                a[1] += tmp[1];
            }
        }

//        if (computeDistance(lp, fp) <= LO) {
//            double[] tmp = computeOrigin(lp, fp);
//            a[0] += tmp[0];
//            a[1] += tmp[1];
//        }

        return a;
    }

    static double[] computeAcceleration(FollowerPlane vp, LeaderPlane lp, FollowerPlane fp, List<FollowerPlane> fps) {

        double[] a = {.0, .0};
        if (computeDistance(vp, fp) <= LA) {
            double[] tmp = computeAttract(vp, fp);
            a[0] += tmp[0];
            a[1] += tmp[1];
        }
        if (computeDistance(lp, fp) <= LR2) {
            double[] tmp = computeRepel1(lp, fp);
            a[0] += tmp[0];
            a[1] += tmp[1];
        }
        for (FollowerPlane p : fps) {
            if (p.id != fp.id && computeDistance(fp, p) <= LR2) {
                double[] tmp = computeRepel1(fp, p);
                a[0] += tmp[0];
                a[1] += tmp[1];
            }
        }
        if (computeDistance(lp, fp) <= LO) {
            double[] tmp = computeOrigin(lp, fp);
            a[0] += tmp[0];
            a[1] += tmp[1];
        }

        return a;
    }

    private static double[] computeAttract(Plane vp, FollowerPlane fp) {

        double distance = computeDistance(vp, fp);
        if (distance < 0.000000001) return new double[]{.0, .0};
//        double force = (Constants.CA/(double)LA)*Math.exp(1*distance/(double)LA);
        double force = Constants.CA*Math.pow(distance, 2);
        double sin = (vp.y-fp.y)/distance;
        double cos = (vp.x-fp.x)/distance;
        double a0 = force/Constants.MASS;
        double[] a = new double[2];
        a[0] = a0*cos;
        a[1] = a0*sin;
        return a;
    }

    private static double[] computeRepel1(Plane lp, Plane fp) {

        double distance = computeDistance(lp, fp);
//        double force = (Constants.CR/LR1)*Math.exp(-1*distance/LR1);
        double force = Constants.CR*Math.pow(1/distance, 2);
        double cos = (fp.x-lp.x)/distance;
        double sin = (fp.y-lp.y)/distance;
        double a = force/Constants.MASS;
        return new double[]{a*cos, a*sin};
    }

    private static double[] computeRepel2(Plane lp, Plane fp) {

        double distance = computeDistance(lp, fp);
//        double force = (Constants.CR/LR2)*Math.exp(-1*distance/LR2);
        double force = Constants.CR*Math.pow(1/distance, 2);
        double cos = (fp.x-lp.x)/distance;
        double sin = (fp.y-lp.y)/distance;
        double a = force/Constants.MASS;
        return new double[]{a*cos, a*sin};
    }

    private static double[] computeOrigin2(LeaderPlane lp, FollowerPlane fp) {

        double velocity = computeVelocity(lp, fp);
//        double force = (0.05*Constants.CO/(double)LO)*Math.exp(1*velocity/(double)LO);
        double force = Constants.CO*Math.pow(velocity, 2);
        if (velocity < 0.000000001) return new double[]{.0, .0};
        double cos = (lp.vx-fp.vx)/velocity;
        double sin = (lp.vy-fp.vy)/velocity;
        double a = force/Constants.MASS;
        return new double[]{a*cos, a*sin};
    }

    private static double[] computeOrigin(LeaderPlane lp, FollowerPlane fp) {

        double velocity = computeVelocity(lp, fp);
//        double force = (Constants.CO/(double)LO)*Math.exp(1*velocity/(double)LO);
        double force = Constants.CO*Math.pow(velocity, 2);
        if (velocity < 0.000000001) return new double[]{.0, .0};
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

    static List<FollowerPlane> cloneVirtual(List<FollowerPlane> pl) {

        List<FollowerPlane> vp = new ArrayList<>();
        for (Plane p : pl) vp.add(new FollowerPlane(p.x, p.y, p.vx, p.vy, p.id));
        return vp;
    }
}
