package com.silver.uav;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * inner_init
 * +-+-+-+-+-+-+-+-+-+
 * simulation time
 * sample frequency per second
 * leader plane
 * +-+-+-+-+-+-+-+-+-+
 */
public class OuterInit {

    private static int lid = 1;

    static Map<String, Object> readText() {
        Map<String, Object> map = new HashMap<>();

        try (BufferedReader br = new BufferedReader(new FileReader("C:\\Users\\Yuche\\IdeaProjects\\UAV-Formation\\src\\main\\resources\\outer_init"))) {

            int n = Constants.N_OF_GROUP;
            map.put("time", Integer.valueOf(br.readLine()));
            map.put("sample", Integer.valueOf(br.readLine()));

            List<List<Plane>> groups  = new ArrayList<>();
            for (int i = 0; i < n; i++) groups.add(generateGroups(br.readLine()));
            map.put("groups", groups);

        } catch (IOException ioe) {
            ioe.printStackTrace();
        }

        return map;
    }

    private static List<Plane> generateGroups(String str) {

        List<Plane> group = new ArrayList<>();
        String[] s = str.split(" ");
        Double x = Double.valueOf(s[0]);
        Double y = Double.valueOf(s[1]);
        Double vx = Double.valueOf(s[2]);
        Double vy = Double.valueOf(s[3]);
        LeaderPlane lp = new LeaderPlane(x, y, vx, vy, .0, lid++);

        group.add(lp);

        for (int i = 0; i < Constants.N_IN_GROUP; i++) {
            switch (i) {
                case 0 :
                    group.add(new FollowerPlane(x+Constants.RAI, y, vx, vy, i+1));
                    break;
                case 1 :
                    group.add(new FollowerPlane(x+0.5*Constants.RAI, y+0.5*Math.sqrt(3)*Constants.RAI, vx, vy, i+1));
                    break;
                case 2 :
                    group.add(new FollowerPlane(x-0.5*Constants.RAI, y+0.5*Math.sqrt(3)*Constants.RAI, vx, vy, i+1));
                    break;
                case 3 :
                    group.add(new FollowerPlane(x-Constants.RAI, y, vx, vy, i+1));
                    break;
                case 4 :
                    group.add(new FollowerPlane(x-0.5*Constants.RAI, y-0.5*Math.sqrt(3)*Constants.RAI, vx, vy, i+1));
                    break;
                case 5 :
                    group.add(new FollowerPlane(x+0.5*Constants.RAI, y-0.5*Math.sqrt(3)*Constants.RAI, vx, vy, i+1));
                    break;
            }

        }

        return group;
    }
}
