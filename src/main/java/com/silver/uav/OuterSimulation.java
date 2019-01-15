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
        int timer = 1;

        /* ***************  compute dt  *************** */
//        List<Point> points = new ArrayList<>();
//
//        for (LeaderPlane lp : groups) {
//            points.add(new Point(lp.x, lp.y));
//        }
//
//        DTNoBigPoints dt = new DTNoBigPoints(points);
//        List<Triangle> tl = dt.triangles;

        Map<LeaderPlane, Set<LeaderPlane>> memory = new HashMap<>();
        OuterController.initMemory(memory, groups);

        /* ***************  start  *************** */
        for (int i = 0; i < time * sample; i++) {
            double f = 1/(double) sample;

            /* ***************  leader plane  *************** */
            LeaderPlane leader = groups.get(0);
            leader.x = leader.x1;
            leader.y = leader.y1;
            leader.vx = leader.vx1;
            leader.vy = leader.vy1;
            double v = 10;
            if (i < 25*sample) {
                theta = 0;
            } else if (i >= 25*sample && i < 55*sample){
                theta += Math.PI/(180*sample);
            } else if (i >= 55*sample && i < 85*sample) {
                theta -= Math.PI/(180*sample);
            } else if (i >= 85*sample && i < 90*sample) {
                theta = 0;
            } else if (i >=90*sample && i < 120*sample) {
                theta -= Math.PI/(180*sample);
            } else if (i >= 120*sample && i < 150*sample) {
                theta += Math.PI/(180*sample);
            } else {
                theta = 0;
            }
            leader.x1 += f*v*Math.cos(theta);
            leader.y1 += f*v*Math.sin(theta);
            leader.vx1 = v*Math.cos(theta);
            leader.vy1 = v*Math.sin(theta);

            /* ***************  Outer Group Compute  *************** */
            if (timer++ <= 5) {
                for (int j = 1; j < Constants.N_OF_GROUP; j++) {
                    LeaderPlane lp = groups.get(j);
                    OuterController.processOuter2_1(lp, f);
                }
            } else {
                for (int j = 1; j < Constants.N_OF_GROUP; j++) {
                    LeaderPlane lp = groups.get(j);
                    OuterController.processOuter2(lp, memory.get(lp), f);
                }
                timer = 1;
            }

            /* ***************  current plane x,y output  *************** */
            OuterController.printCurrentPlane(i/(double)sample, groups);
            OuterController.printToText(groups);
            OuterController.printDistance(groups);
        }

    }
}
