package ru.mcst.RobotGroup.PathsLinking;

import ru.mcst.RobotGroup.PathsFinding.Hypervisor;

import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Random;

/**
 * Created by bocharov_n on 02.03.16.
 */
class Tracker extends Thread{
    private Camera camera;
    private ArrayList<RobotTrajectory> trajectories;
    private ArrayList<Integer> visibleRobots;
    private ArrayList<Color> colors;
    private ArrayList<RobotTrajectory> robotsTrajectories;
    private boolean markForClear;
//    private int iterationsCounter;
    //private ArrayList<Integer> robotsTrajectoriesDirections;

    public Tracker(Camera camera){
        super();
        this.camera = camera;
        trajectories = new ArrayList<RobotTrajectory>();
        visibleRobots = new ArrayList<Integer>();
        colors = new ArrayList<Color>();
        robotsTrajectories = new ArrayList<RobotTrajectory>();
        this.setName("Camera " + camera.getIndex());
        markForClear = false;
    }

    public void run(){
        System.out.println("Tracker start");
        while (camera.isExist()){
            if(markForClear){
                markForClear = false;
                trajectories.clear();
                robotsTrajectories.clear();
                visibleRobots.clear();
            }
            ArrayList<double[]> allCoordinates = Hypervisor.getAllCoordinates();
            ArrayList<Double> speeds = Hypervisor.getSpeeds();
            ArrayList<Long> times = Hypervisor.getUpdateTimes();
            if(colors.size() < allCoordinates.size()){
                for(int i = 0; i < allCoordinates.size(); i++){
                    Random rand = new Random();
                    float r = rand.nextFloat();
                    float g = rand.nextFloat();
                    float b = rand.nextFloat();
                    colors.add(new Color(r, g, b));
                }
            }
            if(robotsTrajectories.size() < allCoordinates.size()){
                for(int i = 0; i < allCoordinates.size(); i++){
                    robotsTrajectories.add(new RobotTrajectory());
                    //robotsTrajectoriesDirections.add(0);
                }
            }

            int[] mapSize = Hypervisor.getMapSize();
            if(mapSize != null && MapUnderlay.getMapLayer() != null && MapUnderlay.getTrajectoriesLayer() != null) {
                BufferedImage trajectories = new BufferedImage(mapSize[0],mapSize[1], BufferedImage.TYPE_INT_ARGB);
                Graphics2D g2d = trajectories.createGraphics();
                g2d.setComposite(AlphaComposite.Clear);
                g2d.fillRect(0, 0, trajectories.getWidth(), trajectories.getHeight());
                g2d.setComposite(AlphaComposite.Src);
                g2d.setColor(Color.RED);

                ArrayList<Integer> currentVisibleRobots = new ArrayList<Integer>();

                for (int i = 0; i < allCoordinates.size(); i++){
                    double[] coord = allCoordinates.get(i);
                    Point2D currentPoint = new Point2D.Double(coord[0], coord[1]);
                    if (camera.isVisible(currentPoint) && speeds.get(i) > 0) {
                        currentVisibleRobots.add(i);
                        g2d.setColor(colors.get(i));
                        g2d.fillOval((int) coord[0] - 3, (int) coord[1] - 3, 6, 6);
                        if(visibleRobots.indexOf(i) == -1){
                            robotsTrajectories.get(i).getPoints().clear();   //Очищаем прошлую траекторию робота
                            robotsTrajectories.get(i).setDirection(2); // Выставляем флаг входа\выхода в положение "только вход"
                            System.out.println("Robot " + i + " entered scope");
                        }
                        Point2D prevPoint;
                        if(robotsTrajectories.get(i).getPoints().size() > 0)
                            prevPoint = robotsTrajectories.get(i).getPoints().get(robotsTrajectories.get(i).getPoints().size() - 1);
                        else prevPoint = new Point2D.Double(-1, -1);            //impossible point
                        if(!currentPoint.equals(prevPoint)) {
                            robotsTrajectories.get(i).getPoints().add(currentPoint);  //Добавляем его координату
                            robotsTrajectories.get(i).getSpeeds().add(speeds.get(i));           //И скорость
                            robotsTrajectories.get(i).getTimes().add(times.get(i));                                     //И текущее время
                            //Проверяем одновременную видимость с нескольких камер
                            for (Camera curCamera : TrackingSystem.getCameraList()) {
                                if (curCamera.getTracker().isRobotVisibleNow(i) &&
                                        robotsTrajectories.get(i).getConnectedTrajectories().indexOf(curCamera.getTracker().getRobotsTrajectories().get(i)) == -1 &&
                                        curCamera != camera) {
                                    if (robotTrajectoryLength(i) > curCamera.getTracker().robotTrajectoryLength(i)) {
                                        robotsTrajectories.get(i).getNext().add(curCamera.getTracker().getRobotsTrajectories().get(i));
                                    } else {
                                        robotsTrajectories.get(i).getPrev().add(curCamera.getTracker().getRobotsTrajectories().get(i));
                                    }
                                    System.out.println("double vision");
                                    robotsTrajectories.get(i).getConnectedTrajectories().add(curCamera.getTracker().getRobotsTrajectories().get(i));
                                }
                            }
                        }
                    }
                }
                for(int i:visibleRobots){
                    if (currentVisibleRobots.indexOf(i) == -1){
                        RobotTrajectory trajectory = robotsTrajectories.get(i);
                        System.out.println("Robot " + i + " exited scope");
                        if(robotsTrajectories.get(i).getDirection() == 2)
                            trajectory.setDirection(1);
                        if(robotsTrajectories.get(i).getDirection() == 0)
                            trajectory.setDirection(3);
                        this.trajectories.add(trajectory);
                        robotsTrajectories.set(i, new RobotTrajectory());
                        robotsTrajectories.get(i).getPoints().clear();
                        robotsTrajectories.get(i).setDirection(0);
                        Random rand = new Random();
                        float r = rand.nextFloat();
                        float g = rand.nextFloat();
                        float b = rand.nextFloat();
                        colors.set(i, new Color(r, g, b));
                    }
                }
                visibleRobots = currentVisibleRobots;
                g2d.dispose();
                MapUnderlay.changeTrajectoriesLayer(trajectories);
                GUI.getMapPanel().repaint();
            }
            try{
                sleep(25);
            }
            catch(InterruptedException ex){
                System.out.println("Some error in thread sleep");
                ex.printStackTrace();
            }

        }
        System.out.println("Tracker end");
    }

    public void finishAllTrajectories(){
        for(RobotTrajectory robotTrajectory:robotsTrajectories){
            if(!robotTrajectory.getPoints().isEmpty()){
                trajectories.add(new RobotTrajectory(robotTrajectory));
            }
        }
    }

    public int getVisibleRobotsCount() {
        return visibleRobots.size();
    }

    public boolean isRobotVisibleNow(int index){
        return visibleRobots.indexOf(index) >= 0;
    }

    public int robotTrajectoryLength(int index) {
        return isRobotVisibleNow(index) ? robotsTrajectories.get(index).getPoints().size() : -1;
    }

    public ArrayList<RobotTrajectory> getRobotsTrajectories() {
        return robotsTrajectories;
    }

    public ArrayList<RobotTrajectory> getTrajectories() {
        return trajectories;
    }

    public void setMarkForClear(boolean markForClear) {
        this.markForClear = markForClear;
    }
}
