package ru.mcst.RobotGroup.PathsFinding;

import java.awt.*;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.*;
import java.util.List;

/**
 * Created by sapachev_i on 12/9/15.
 */
public class Hypervisor {
    private static List<Robot> robots=new ArrayList<Robot>();
    private static boolean logging=false;

    public Hypervisor() {
    }

    public Hypervisor(List<Robot> robots) {
        this.robots=robots;
    }

    public static void sendMapChangedSignal(boolean value) {
        for(Robot r:robots) {
            r.setMapChangedSignal(value);
        }
    }

    public static Robot checkRobotsOnWay(Robot robot, Link link, double passed) {
        for (Robot r : robots) {
            if (r != robot) {
                    if (Link.isSegmentsBlockedByRobot(link.getSegments(),
                            (int) r.getX(), (int) r.getY(), r.getAzimuth(), robot.getMap().getScale(), 0, passed)) {
                        return r;
                    }

            }
        }
        return null;
    }

    public static Robot checkRobotsOnWay(Robot robot, List<Link> links, double passed) {
        if(links.isEmpty())
            return null;
        Link currentLink=links.get(0);

        for(Robot r:robots){
            if (r != robot) {
                for(Link link: links) {
                    if(link==currentLink) {
                        if (Link.isSegmentsBlockedByRobot(link.getSegments(),
                                (int) r.getX(), (int) r.getY(), r.getAzimuth(), robot.getMap().getScale(), 0, passed)) {
                            return r;
                        }
                    } else {
                        if (Link.isSegmentsBlockedByRobot(link.getSegments(),
                                (int) r.getX(), (int) r.getY(), r.getAzimuth(), robot.getMap().getScale(), 0, 0)) {
                            return r;
                        }
                    }
                }
            }
        }
        return null;
    }

    public static Robot checkRobotInCoordinates(int x, int y, int robotSize, Robot robot) {
        for(Robot r: robots) {
            if(r!=robot) {
                if (Math.abs(r.getX() - x) <= robotSize && Math.abs(r.getY() - y) <= robotSize) {
                    return r;
                }
            }
        }
        return null;
    }

    public synchronized static boolean isPointOccupiedAsFinish(double x, double y, Robot exeption) {
        for(Robot robot: robots) {
            if(robot!=exeption) {
                Node destination=robot.getRealDestination();
                if(destination!=null) {
                    if (destination.getX() == x && destination.getY() == y) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    public static void startLog() {
        logging=true;
        Thread thread=new Thread(new Runnable() {
            @Override
            public void run() {
                // create files
                PrintWriter writers[]=new PrintWriter[robots.size()];
                for(int i=0; i<robots.size(); ++i) {
                    try {
                        writers[i]=new PrintWriter("robot_"+i);
                    } catch (FileNotFoundException e) {
                        e.printStackTrace();
                    }
                }
                for(int i=0; i<robots.size(); ++i) {
                    Robot r=robots.get(i);
                    writers[i].println(r.getMap().getWidth() + " " + r.getMap().getHeight());
                    ++i;
                }
                // while not stopped - log MOVING robots
                while (logging) {
                    for(int i=0; i<robots.size(); ++i) {
                        Robot r=robots.get(i);
                        if(r.getSpeed()>0) {
                            writers[i].println((int)r.getX() + " " + (int)r.getY());
                        }
                        ++i;
                    }
                    try {
                        Thread.sleep(25);        //40 fps
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }

                for(PrintWriter w:writers)
                    w.close();
            }
        });
        thread.start();
    }

    public static void stopLog() {
        logging=false;
    }

    // returns {width, height}
    public static int[] getMapSize() {
        if(robots.size()==0)
            return null;
        MapInfo map=robots.get(0).getMap();
        if(map.getImage()==null)
            return null;
        int result[]=new int[2];
        result[0]=map.getWidth();
        result[1]=map.getHeight();
        return result;
    }

    public static Image getMapImage() {
        if(robots.size()==0)
            return null;
        return robots.get(0).getMap().getImage();
    }

    public static ArrayList<double[]> getAllCoordinates() {
        ArrayList<double[]> result = new ArrayList<double[]>();

        for(Robot r:robots) {
            result.add(r.getCachedCoordinates());
        }
        return result;
    }

    public static ArrayList<Double> getSpeeds() {
        ArrayList<Double> result = new ArrayList<Double>();
        for (Robot r : robots)
            result.add(r.getCachedSpeed());
        return result;
    }

    public static ArrayList<Long> getUpdateTimes() {
        ArrayList<Long> result=new ArrayList<Long>();
        for(Robot r:robots)
            result.add(r.getUpdateTime());
        return result;
    }
}
