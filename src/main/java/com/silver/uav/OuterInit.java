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
class OuterInit {

    static Map<String, Object> readText() {
        Map<String, Object> map = new HashMap<>();

        try (BufferedReader br = new BufferedReader(new FileReader("C:\\Users\\Yuche\\IdeaProjects\\UAV-Formation\\src\\main\\resources\\outer_init"))) {

            int n = Constants.N_OF_GROUP;
            map.put("time", Integer.valueOf(br.readLine()));
            map.put("sample", Integer.valueOf(br.readLine()));

            List<LeaderPlane> groups  = new ArrayList<>();
            for (int i = 0; i < n; i++) groups.add(generateGroups(br.readLine()));
            map.put("groups", groups);

        } catch (IOException ioe) {
            ioe.printStackTrace();
        }

        return map;
    }

    private static LeaderPlane generateGroups(String str) {

        String[] s = str.split(" ");
        int id = Integer.valueOf(s[0]);
        Double x = Double.valueOf(s[1]);
        Double y = Double.valueOf(s[2]);
        Double vx = Double.valueOf(s[3]);
        Double vy = Double.valueOf(s[4]);
        return new LeaderPlane(x, y, vx, vy, .0, id);
    }
}
