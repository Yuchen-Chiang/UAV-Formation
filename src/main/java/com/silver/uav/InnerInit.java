package com.silver.uav;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

/**
 * inner_init
 * +-+-+-+-+-+-+-+-+-+
 * simulation time
 * sample frequency per second
 * leader plane
 * follower planes
 * +-+-+-+-+-+-+-+-+-+
 */
public class InnerInit {

    private static int fid = 1;

    static Map<String, Object> readText() {

        Map<String, Object> map = new HashMap<>();

        try (BufferedReader br = new BufferedReader(new FileReader("C:\\Users\\Yuche\\IdeaProjects\\UAV-Formation\\src\\main\\resources\\inner_init"))) {

            int n = Constants.NUM_IN_GROUP;
            map.put("time", Integer.valueOf(br.readLine()));
            map.put("sample", Integer.valueOf(br.readLine()));
            map.put("leader", generateLeader(br.readLine()));

            List<FollowerPlane> followerPlanes = new ArrayList<>();
            for (int i = 0; i < n; i++) followerPlanes.add(generateFollowers(br.readLine()));
            map.put("followers", followerPlanes);
        } catch (IOException ioe) {
            ioe.printStackTrace();
        }

        return map;
    }

    private static LeaderPlane generateLeader(String str) {

        String[] s = str.split(" ");
        Double x = Double.valueOf(s[0]);
        Double y = Double.valueOf(s[1]);
        Double vx = Double.valueOf(s[2]);
        Double vy = Double.valueOf(s[3]);

        return new LeaderPlane(x, y, vx, vy, .0, 1);
    }

    private static FollowerPlane generateFollowers(String str) {

        String[] s = str.split(" ");
        Double vx = Double.valueOf(s[2]);
        Double vy = Double.valueOf(s[3]);
        Double x = Double.valueOf(s[0]);
        Double y = Double.valueOf(s[1]);
        return new FollowerPlane(x, y, vx, vy, fid++);
    }
}
