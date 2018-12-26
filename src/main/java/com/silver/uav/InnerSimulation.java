package com.silver.uav;

import java.util.List;
import java.util.Map;

public class InnerSimulation {

    @SuppressWarnings("unchecked")
    public static void main(String[] args) {

        /* ***************  init  *************** */
        Map<String, Object> map = InnerInit.readText();
        int time = (int) map.get("time");
        int sample = (int) map.get("sample");
        LeaderPlane lp = (LeaderPlane) map.get("leader");
        List<FollowerPlane> fps = (List<FollowerPlane>) map.get("followers");

        /* ***************  start  *************** */
        for (int i = 0; i < time * sample; i++) {
            double f = 1/(double)sample;

            /* ***************  leader plane computing  *************** */
            lp.x = lp.x1;
            lp.y = lp.y1;
            lp.x1 += f*lp.vx;
            lp.y1 += f*lp.vy;

            /* ***************  follower plane computing  *************** */
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

            /* ***************  current plane x,y output  *************** */
            InnerController.printCurrentPlane(i/(double)sample, lp, fps);
        }
    }
}
