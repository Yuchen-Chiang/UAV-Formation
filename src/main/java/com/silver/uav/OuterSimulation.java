package com.silver.uav;

import ProGAL.geom2d.Point;
import ProGAL.geom2d.delaunay.DTNoBigPoints;
import ProGAL.geom2d.delaunay.Triangle;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class OuterSimulation {

    @SuppressWarnings("unchecked")
    public static void main(String[] args) {

        /* ***************  init  *************** */
        Map<String, Object> map = OuterInit.readText();
        int time = (int) map.get("time");
        int sample = (int) map.get("sample");
        List<List<Plane>> groups = (List<List<Plane>>) map.get("groups");

        /* ***************  start  *************** */
        for (int i = 0; i < time * sample; i++) {
            double f = 1/(double) sample;

            List<Point> points = new ArrayList<>();

            for (List<Plane> group : groups) {
                LeaderPlane lp = (LeaderPlane) group.get(0);
                points.add(new Point(lp.x, lp.y));
            }

            DTNoBigPoints dt = new DTNoBigPoints(points);
            List<Triangle> tl = dt.triangles;

            for (List<Plane> group : groups) {
                /* ***************  Outer Group Compute  *************** */
                LeaderPlane lp = (LeaderPlane) group.get(0);
                OuterController.processOuter(lp, tl, f);

                /* ***************  Inner Group Compute  *************** */
                List<FollowerPlane> fps = new ArrayList<>();
                for (int j = 1; j <= Constants.N_IN_GROUP; j++) fps.add((FollowerPlane) group.get(j));
                for (FollowerPlane fp : fps) {
                    double[] a = InnerController.computeAcceleration(lp, fp, fps);

                    fp.x = fp.x1;
                    fp.y = fp.y1;
                    fp.vx = fp.vx1;
                    fp.vy = fp.vy1;
                    fp.ax = fp.ax1;
                    fp.ay = fp.ay1;

                    fp.x1 += fp.vx*f + 0.5*fp.ax*f*f;
                    fp.y1 += fp.vy*f + 0.5*fp.ay*f*f;
                    fp.vx1 += fp.ax*f;
                    fp.vy1 += fp.ay*f;
                    fp.ax1 = a[0];
                    fp.ay1 = a[1];
                }
            }

            /* ***************  current plane x,y output  *************** */
            OuterController.printCurrentPlane(i/(double)sample, groups);
        }

    }
}
