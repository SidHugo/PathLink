package ru.mcst.RobotGroup.PathsLinking;

import com.sun.javafx.geom.Vec2d;

import java.awt.*;
import java.awt.geom.Point2D;
import java.util.HashSet;

/**
 * Created by bocharov_n on 19.02.16.
 */
class InOutVector {
    public static final int IN  = 0,
                            OUT = 1;

    private RobotTrajectory robotTrajectory;
    private Point2D startPoint, endPoint;
    private double speed;
    private double acceleration;
    private long time;
    private int orientation;
    private HashSet<InOutVector> prev, next;
    long  startTime, endTime;

    public InOutVector(){
        robotTrajectory = null;
        startPoint = null;
        endPoint = null;
        prev = new HashSet<InOutVector>();
        next = new HashSet<InOutVector>();
    }

    public InOutVector(RobotTrajectory robotTrajectory, int orientation){
        int startIndex = 0, endIndex = 1;
        double speed = 0;
        switch (orientation){
            case IN:
                startIndex = 0;
                endIndex = 1;
                this.startPoint = robotTrajectory.getPoints().get(startIndex);
                this.endPoint = robotTrajectory.getPoints().get(endIndex);
                this.speed = 1000 * Math.sqrt(Math.pow(endPoint.getX() - startPoint.getX(), 2) +
                        Math.pow(endPoint.getY() - startPoint.getY(), 2)) /
                        (robotTrajectory.getTimes().get(endIndex) - robotTrajectory.getTimes().get(startIndex));
                speed = 1000 * Math.sqrt(Math.pow(robotTrajectory.getPoints().get(endIndex + 1).getX() - robotTrajectory.getPoints().get(endIndex).getX(), 2) +
                        Math.pow(robotTrajectory.getPoints().get(endIndex + 1).getY() - robotTrajectory.getPoints().get(endIndex).getY(), 2)) /
                        (robotTrajectory.getTimes().get(endIndex + 1) - robotTrajectory.getTimes().get(endIndex));
                this.acceleration = 1000 * (speed - this.speed) / (robotTrajectory.getTimes().get(endIndex + 1) - robotTrajectory.getTimes().get(endIndex));
                break;
            case OUT:
                startIndex = robotTrajectory.getPoints().size() - 2;
                endIndex = robotTrajectory.getPoints().size() - 1;
                this.startPoint = robotTrajectory.getPoints().get(startIndex);
                this.endPoint = robotTrajectory.getPoints().get(endIndex);
                this.speed = 1000 * Math.sqrt(Math.pow(endPoint.getX() - startPoint.getX(), 2) +
                        Math.pow(endPoint.getY() - startPoint.getY(), 2)) /
                        (robotTrajectory.getTimes().get(endIndex) - robotTrajectory.getTimes().get(startIndex));
                speed = 1000 * Math.sqrt(Math.pow(robotTrajectory.getPoints().get(startIndex).getX() - robotTrajectory.getPoints().get(startIndex - 1).getX(), 2) +
                        Math.pow(robotTrajectory.getPoints().get(startIndex).getY() - robotTrajectory.getPoints().get(startIndex - 1).getY(), 2)) /
                        (robotTrajectory.getTimes().get(startIndex) - robotTrajectory.getTimes().get(startIndex - 1));
                this.acceleration = 1000 * (this.speed - speed) / (robotTrajectory.getTimes().get(startIndex) - robotTrajectory.getTimes().get(startIndex - 1));
                break;
        }
//        this.speed = robotTrajectory.getSpeeds().get(startIndex);
        this.time = robotTrajectory.getTimes().get(startIndex);
        this.startTime = robotTrajectory.getTimes().get(startIndex);
        this.endTime = robotTrajectory.getTimes().get(endIndex);
        this.robotTrajectory = robotTrajectory;
        this.orientation = orientation;
        prev = new HashSet<InOutVector>();
        next = new HashSet<InOutVector>();
    }

    public void drawVector(Graphics2D g2d, boolean isFilled, boolean isBorder){
        g2d.setComposite(AlphaComposite.Src);
        g2d.setColor(Color.GRAY);
        //TODO:make here a triangle
        if (isFilled) g2d.fillRoundRect((int)(orientation == 0 ? startPoint.getX() : endPoint.getX()) - 5,
                (int)(orientation == 0 ?  startPoint.getY() : endPoint.getY()) - 5, 10, 10, 2, 2);
        else g2d.drawRoundRect((int)(orientation == 0 ? startPoint.getX() : endPoint.getX()) - 5,
                (int)(orientation == 0 ?  startPoint.getY() : endPoint.getY()) - 5, 10, 10, 2, 2);
        if (isBorder){
            g2d.setColor(Color.BLACK);
            g2d.drawRoundRect((int)(orientation == 0 ? startPoint.getX() : endPoint.getX()) - 5,
                    (int)(orientation == 0 ?  startPoint.getY() : endPoint.getY()) - 5, 10, 10, 2, 2);
        }
    }

    public double getAzimuth(){
        double x = endPoint.getX() - startPoint.getX(),
                y = endPoint.getY() - startPoint.getY();
        return ( x == 0 ) ? 90 : (x < 0 && y > 0) ? (180 + Math.toDegrees(Math.atan(y / x))) : Math.toDegrees(Math.atan(y / x));
    }

    public double getNormal(){
        double c = startPoint.getX() * endPoint.getY() - endPoint.getX() * startPoint.getY(),
                a = endPoint.getX() - startPoint.getX(),
                b = startPoint.getY() - endPoint.getY();
        return Math.abs(c / Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2)));
    }

    public boolean isPotentialFollowerTo(InOutVector vector){
//        long startTime = System.currentTimeMillis();
        double POSSIBLE_ANGLE = 90;

        double n = 15;          //rotation degrees
        double x = endPoint.getX() - startPoint.getX(), y = endPoint.getY() - startPoint.getY();
        
        Vec2d wayVector = new Vec2d(vector.getStartPoint().getX() - endPoint.getX(), vector.getStartPoint().getY() - endPoint.getY());
        Vec2d sectorStart = new Vec2d(x * Math.cos(n) - y * Math.sin(n), x * Math.sin(n) + y * Math.cos(n)),
                sectorEnd = new Vec2d(x * Math.cos(-n) - y * Math.sin(-n), x * Math.sin(-n) + y * Math.cos(-n));
        double wayVectorLength = Math.sqrt(Math.pow(wayVector.x, 2) + Math.pow(wayVector.y, 2)),
                sectorStartLength = Math.sqrt(Math.pow(sectorStart.x, 2) + Math.pow(sectorStart.y, 2)),
                sectorEndLength = Math.sqrt(Math.pow(sectorEnd.x, 2) + Math.pow(sectorEnd.y, 2));
        sectorStart = new Vec2d(sectorStart.x * wayVectorLength / sectorStartLength, sectorStart.y * wayVectorLength / sectorStartLength);
        sectorEnd = new Vec2d(sectorEnd.x * wayVectorLength / sectorEndLength, sectorEnd.y * wayVectorLength / sectorEndLength);
        double distance = Math.sqrt(Math.pow(endPoint.getX() - vector.getStartPoint().getX(), 2) +
                Math.pow(endPoint.getY() - vector.getStartPoint().getY(), 2)),
               possibleDistance = (this.speed + vector.speed) / 2 * ((double)(vector.time - this.time) / 1000);
//        System.out.println(distance + " " + possibleDistance);
        boolean isInReachableDistance = (distance < possibleDistance * 1.2 && distance > possibleDistance * 0.7) || (
                distance < possibleDistance + 5 && distance > possibleDistance - 5);
        boolean isAzimuthCorrect = Math.abs(this.getAzimuth() - vector.getAzimuth()) < POSSIBLE_ANGLE;
        boolean isInSector = !areClockwise(sectorStart, wayVector) && areClockwise(sectorEnd, wayVector);
//        System.out.println(isInReachableDistance);
//        System.out.println(isAzimuthCorrect);
//        System.out.println(isInSector);
//        System.out.println(distance + " " + possibleDistance);
//        System.out.println("isPotentialFollower time(ms): " + (System.currentTimeMillis() - startTime));
        return isInSector && isInReachableDistance && isAzimuthCorrect;
    }

    private boolean areClockwise(Vec2d v1, Vec2d v2){
        return -v1.x * v2.y + v1.y * v2.x <= 0;
    }

    public double getX(){
        return orientation == IN ? startPoint.getX() : endPoint.getX();
    }

    public double getY(){
        return orientation == IN ? startPoint.getY() : endPoint.getY();
    }

    public RobotTrajectory getRobotTrajectory() {
        return robotTrajectory;
    }

    public Point2D getStartPoint() {
        return startPoint;
    }

    public Point2D getEndPoint() {
        return endPoint;
    }

    public double getSpeed() {
        return speed;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public long getTime() {
        return time;
    }

    public int getOrientation() {
        return orientation;
    }

    public HashSet<InOutVector> getNext() {
        return next;
    }

    public HashSet<InOutVector> getPrev() {
        return prev;
    }
}
