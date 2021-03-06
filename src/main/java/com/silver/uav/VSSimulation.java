package com.silver.uav;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * inner_init
 * +-+-+-+-+-+-+-+-+-+
 * simulation time
 * sample frequency per second
 * virtual leader
 * planes
 * +-+-+-+-+-+-+-+-+-+
 */
public class VSSimulation {

    private static int id = 0;
    private static double theta = .0;
    private static double[] vn = new double[]{.0, .0, .0, .0, .0, .0, .0};
    private static double[] fn = new double[]{.0, .0, .0, .0, .0, .0, .0};
    private static double[] exn = new double[]{.0, .0, .0, .0, .0, .0, .0};
    private static double[] eyn = new double[]{.0, .0, .0, .0, .0, .0, .0};

    @SuppressWarnings("unchecked")
    public static void main(String[] args) {

        /* ***************  init  *************** */
        Map<String, Object> map = readText();
        int time = (int) map.get("time");
        int sample = (int) map.get("sample");
        Plane leader = (Plane) map.get("leader");
        List<Plane> pl = (List<Plane>) map.get("planes");
        List<Plane> vp = cloneVirtual(pl);

        /* ***************  start  *************** */
        for (int i = 0; i < time*sample; i++) {
            double f = 1/(double)sample;

            /* ***************  virtual leader & virtual plane  *************** */
            leader.x = leader.x1;
            leader.y = leader.y1;
            double v = 10;
            if (i < time*sample/10) {
                theta = 0;
            } else {
                theta += Math.PI/3600;
                if (theta > Math.PI) theta = Math.PI;
            }
            leader.x1 += f*v*Math.cos(theta);
            leader.y1 += f*v*Math.sin(theta);

            for (int j = 0; j < Constants.N_IN_GROUP; j++) {
                Plane p = vp.get(j);
                p.x = p.x1;
                p.y = p.y1;
                switch (j) {
                    case 0 :
                        p.x1 = leader.x1;
                        p.y1 = leader.y1;
                        break;
                    case 1 :
                        p.x1 = leader.x1-Math.cos(theta)*10-Math.sin(theta)*10;
                        p.y1 = leader.y1-Math.sin(theta)*10+Math.cos(theta)*10;
                        break;
                    case 2 :
                        p.x1 = leader.x1-Math.cos(theta)*10+10*Math.sin(theta);
                        p.y1 = leader.y1-Math.sin(theta)*10-10*Math.cos(theta);
                        break;
                }

            }

            /* ***************  plane computing  *************** */
            for (int j = 0; j < Constants.N_IN_GROUP; j++) {
                double ex = vp.get(j).x - pl.get(j).x;
                double ey = vp.get(j).y - pl.get(j).y;
                double exf = ex*Math.cos(theta)+ey*Math.sin(theta);
                double eyf = -ex*Math.sin(theta)+ey*Math.cos(theta);

                double delta_v = (Constants.K_PX+Constants.K_IX)*exf-Constants.K_PX*exn[j];
                double delta_f = (Constants.K_PY+Constants.K_IY)*eyf-Constants.K_PY*eyn[j];

                exn[j] = exf;
                eyn[j] = eyf;

                vn[j] += delta_v;
                fn[j] += delta_f;

                pl.get(j).x = pl.get(j).x1;
                pl.get(j).y = pl.get(j).y1;
                pl.get(j).x1 += f*(v+vn[j])*Math.cos(theta+fn[j]);
                pl.get(j).y1 += f*(v+vn[j])*Math.sin(theta+fn[j]);
            }

            /* ***************  print  *************** */
            print(i/(double)sample, leader, pl);
            printToText(pl);
            computeError(pl);
        }

    }

    private static void computeError(List<Plane> pl) {

        Plane p0 = pl.get(0);
        Plane p1 = pl.get(1);
        Plane p2 = pl.get(2);

        double dist01 = Math.sqrt(Math.pow(p1.x-p0.x, 2)+Math.pow(p1.y-p0.y, 2));
        double dist02 = Math.sqrt(Math.pow(p2.x-p0.x, 2)+Math.pow(p2.y-p0.y, 2));
        double dist12 = Math.sqrt(Math.pow(p2.x-p1.x, 2)+Math.pow(p2.y-p1.y, 2));

        try (RandomAccessFile file1 = new RandomAccessFile("C:\\Users\\Yuche\\Documents\\MATLAB\\vs\\vs_dist_01", "rw");
             RandomAccessFile file2 = new RandomAccessFile("C:\\Users\\Yuche\\Documents\\MATLAB\\vs\\vs_dist_02", "rw");
             RandomAccessFile file3 = new RandomAccessFile("C:\\Users\\Yuche\\Documents\\MATLAB\\vs\\vs_dist_12", "rw")) {
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

    private static void printToText(List<Plane> pl) {

        for (int i = 0; i < pl.size(); i++) {
            Plane p = pl.get(i);
            try (RandomAccessFile fileX = new RandomAccessFile("C:\\Users\\Yuche\\Documents\\MATLAB\\vs\\vs_"+i+"_x", "rw");
                RandomAccessFile fileY = new RandomAccessFile("C:\\Users\\Yuche\\Documents\\MATLAB\\vs\\vs_"+i+"_y", "rw")) {
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

    private static void print(double simulationTime, Plane leader, List<Plane> pl) {

        System.out.println("simulation time =" + simulationTime);
        System.out.println("virtual leader plane (" + leader.x + ", " + leader.y + ")");
        for (Plane p : pl) System.out.println("follower plane " + p.id + " (" + p.x + ", " + p.y + ")");
        System.out.println("--------------------------------------");

    }

    private static List<Plane> cloneVirtual(List<Plane> pl) {

        List<Plane> vp = new ArrayList<>();
        for (Plane p : pl) vp.add(new Plane(p.x, p.y, p.vx, p.vy, p.id));
        return vp;
    }

    private static Map<String, Object> readText() {

        Map<String, Object> map = new HashMap<>();

        try (BufferedReader br = new BufferedReader(new FileReader("C:\\Users\\Yuche\\IdeaProjects\\UAV-Formation\\src\\main\\resources\\vs_init"))) {

            int n = Constants.N_IN_GROUP;
            map.put("time", Integer.valueOf(br.readLine()));
            map.put("sample", Integer.valueOf(br.readLine()));
            map.put("leader", generateLeader(br.readLine()));

            List<Plane> planes = new ArrayList<>();
            for (int i = 0; i < n; i++) planes.add(generatePlane(br.readLine()));
            map.put("planes", planes);
        } catch (IOException ioe) {
            ioe.printStackTrace();
        }

        return map;
    }

    private static Plane generatePlane(String str) {

        String[] s = str.split(" ");
        Double x = Double.valueOf(s[0]);
        Double y = Double.valueOf(s[1]);
        Double vx = Double.valueOf(s[2]);
        Double vy = Double.valueOf(s[3]);

        return new Plane(x, y, vx, vy, id++);
    }

    private static Plane generateLeader(String str) {

        String[] s = str.split(" ");
        Double x = Double.valueOf(s[0]);
        Double y = Double.valueOf(s[1]);
        Double vx = Double.valueOf(s[2]);
        Double vy = Double.valueOf(s[3]);

        return new Plane(x, y, vx, vy, 0);
    }
}
