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

    private static int lid = 0;

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
        Double x = Double.valueOf(s[0]);
        Double y = Double.valueOf(s[1]);
        Double v = Double.valueOf(s[2]);
        Double theta = Double.valueOf(s[3]);
        return new LeaderPlane(x, y, v, theta, .0, lid++);
    }
}
