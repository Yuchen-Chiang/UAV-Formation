package com.silver.uav;

import ProGAL.geom2d.Point;
import ProGAL.geom2d.delaunay.DTNoBigPoints;
import ProGAL.geom2d.delaunay.Triangle;

import java.util.*;

public class OuterSimulation {

    private static double theta = .0;

    @SuppressWarnings("unchecked")
    public static void main(String[] args) {

        /* ***************  init  *************** */
        Map<String, Object> map = OuterInit.readText();
        int time = (int) map.get("time");
        int sample = (int) map.get("sample");
        List<LeaderPlane> groups = (List<LeaderPlane>) map.get("groups");

        /* ***************  compute dt  *************** */
        List<Point> points = new ArrayList<>();

        for (LeaderPlane lp : groups) {
            points.add(new Point(lp.x, lp.y));
        }

        DTNoBigPoints dt = new DTNoBigPoints(points);
        List<Triangle> tl = dt.triangles;

        List<Set<LeaderPlane>> ls = new ArrayList<>();
        for (int i = 1; i < Constants.N_OF_GROUP; i++) {
            LeaderPlane t = groups.get(i);
            Set<LeaderPlane> set = new HashSet<>();
            for (int j = 1; j < Constants.N_OF_GROUP; j++) {
                if (j != i) {
                    LeaderPlane p = groups.get(j);
                    if (OuterController.testAndCheck(tl, t, p)) {
                        set.add(p);
                    }
                }
            }
            ls.add(set);
        }

//        for (int i = 1; i < Constants.N_OF_GROUP; i++) {
//            Set<LeaderPlane> set = ls.get(i-1);
//            System.out.println("-----------------------");
//            System.out.println(i);
//            for (LeaderPlane p : set) System.out.println(p.id);
//            System.out.println("-----------------------");
//        }

        /* ***************  start  *************** */
        for (int i = 0; i < time * sample; i++) {
            double f = 1/(double) sample;

            /* ***************  leader plane  *************** */
            LeaderPlane leader = groups.get(0);
            leader.x = leader.x1;
            leader.y = leader.y1;
            double v = 10;
            if (i < time*sample/15) {
                theta = 0;
            } else {
                theta += Math.PI/3600;
                if (theta > Math.PI/6) theta = Math.PI/6;
            }
            leader.x1 += f*v*Math.cos(theta);
            leader.y1 += f*v*Math.sin(theta);

            /* ***************  Outer Group Compute  *************** */
            for (int j = 1; j < Constants.N_OF_GROUP; j++) {
                LeaderPlane lp = groups.get(j);
                OuterController.processOuter(lp, tl, f);
            }

            /* ***************  current plane x,y output  *************** */
            OuterController.printCurrentPlane(i/(double)sample, groups);
            OuterController.printToText(groups);
            OuterController.printDistance(groups);
        }

    }
}
