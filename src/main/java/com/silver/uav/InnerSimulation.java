package com.silver.uav;

import java.util.List;
import java.util.Map;

public class InnerSimulation {

    private static double theta = .0;

//    @SuppressWarnings("unchecked")
//    public static void main(String[] args) {
//
//        /* ***************  init  *************** */
//        Map<String, Object> map = InnerInit.readText();
//        int time = (int) map.get("time");
//        int sample = (int) map.get("sample");
//        LeaderPlane lp = (LeaderPlane) map.get("leader");
//        List<FollowerPlane> fps = (List<FollowerPlane>) map.get("followers");
//
//        /* ***************  start  *************** */
//        for (int i = 0; i < time * sample; i++) {
//        double f = 1/(double)sample;
//
//        /* ***************  leader plane computing  *************** */
//        lp.x = lp.x1;
//        lp.y = lp.y1;
//        lp.vx = lp.vx1;
//        lp.vy = lp.vy1;
//        double v = 10;
//        if (i < 25*sample) {
//            theta = 0;
//        } else if (i >= 25*sample && i < 55*sample){
//            theta += Math.PI/(180*sample);
//        } else if (i >= 55*sample && i < 85*sample) {
//            theta -= Math.PI/(180*sample);
//        } else if (i >= 85*sample && i < 90*sample) {
//            theta = 0;
//        } else if (i >=90*sample && i < 120*sample) {
//            theta -= Math.PI/(180*sample);
//        } else if (i >= 120*sample && i < 150*sample) {
//            theta += Math.PI/(180*sample);
//        } else {
//            theta = 0;
//        }
//        lp.x1 += f*v*Math.cos(theta);
//        lp.y1 += f*v*Math.sin(theta);
//        lp.vx1 = v*Math.cos(theta);
//        lp.vy1 = v*Math.sin(theta);
//
//        /* ***************  follower plane computing  *************** */
//        for (int j = 0; j < Constants.NUM_IN_GROUP; j++) {
//
//            FollowerPlane fp = fps.get(j);
//
//            double[] a = InnerController.computeAcceleration2(i, lp, fp, fps);
//
//            fp.x = fp.x1;
//            fp.y = fp.y1;
//            fp.vx = fp.vx1;
//            fp.vy = fp.vy1;
//            fp.ax = fp.ax1;
//            fp.ay = fp.ay1;
//
//            fp.x1 += fp.vx*f + 0.5*fp.ax*f*f;
//            fp.y1 += fp.vy*f + 0.5*fp.ay*f*f;
//            fp.vx1 += fp.ax*f;
//            fp.vy1 += fp.ay*f;
//            fp.ax1 = a[0];
//            fp.ay1 = a[1];
//        }
//
//        /* ***************  current plane x,y output  *************** */
//        InnerController.printCurrentPlane(i/(double)sample, lp, fps);
//        InnerController.printToText(lp, fps);
//        InnerController.computeXYError(lp, fps);
//    }
//}

    @SuppressWarnings("unchecked")
    public static void main(String[] args) {

        /* ***************  init  *************** */
        Map<String, Object> map = InnerInit.readText();
        int time = (int) map.get("time");
        int sample = (int) map.get("sample");
        LeaderPlane lp = (LeaderPlane) map.get("leader");
        List<FollowerPlane> fps = (List<FollowerPlane>) map.get("followers");
        List<FollowerPlane> vps = InnerController.cloneVirtual(fps);

        /* ***************  start  *************** */
        for (int i = 0; i < time * sample; i++) {
            double f = 1/(double)sample;

            /* ***************  leader plane computing  *************** */
            lp.x = lp.x1;
            lp.y = lp.y1;
            lp.vx = lp.vx1;
            lp.vy = lp.vy1;
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
            lp.x1 += f*v*Math.cos(theta);
            lp.y1 += f*v*Math.sin(theta);
            lp.vx1 = v*Math.cos(theta);
            lp.vy1 = v*Math.sin(theta);

            /* ***************  virtual plane computing  *************** */
            for (int j = 0; j < Constants.NUM_IN_GROUP; j++) {
                Plane p = vps.get(j);
                p.x = p.x1;
                p.y = p.y1;
                switch (j) {
                    case 0 :
                        p.x1 = lp.x1-Math.cos(theta)*8-Math.sin(theta)*6;
                        p.y1 = lp.y1-Math.sin(theta)*8+Math.cos(theta)*6;
                        break;
                    case 1 :
                        p.x1 = lp.x1-Math.cos(theta)*8+6*Math.sin(theta);
                        p.y1 = lp.y1-Math.sin(theta)*8-6*Math.cos(theta);
                        break;
                }

            }


            /* ***************  follower plane computing  *************** */
            for (int j = 0; j < Constants.NUM_IN_GROUP; j++) {

                FollowerPlane fp = fps.get(j);
                FollowerPlane vp = vps.get(j);

                double[] a = InnerController.computeAcceleration(vp, lp, fp, fps);

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

            /* ***************  current plane x,y output  *************** */
            InnerController.printCurrentPlane(i/(double)sample, lp, fps);
            InnerController.printToText(lp, fps);
            InnerController.computeXYError(lp, fps);
        }
    }
}
