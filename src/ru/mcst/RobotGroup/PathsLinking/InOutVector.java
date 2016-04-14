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
    private long time;
    private int orientation;
    private HashSet<InOutVector> prev, next;
    private Color color;

    public InOutVector(){
        robotTrajectory = null;
        startPoint = null;
        endPoint = null;
        color = null;
        prev = new HashSet<InOutVector>();
        next = new HashSet<InOutVector>();
    }

    public InOutVector(RobotTrajectory robotTrajectory, int orientation){
        switch (orientation){
            case IN:
                this.startPoint = robotTrajectory.getPoints().get(0);
                this.endPoint = robotTrajectory.getPoints().get(1);
                this.speed = robotTrajectory.getSpeeds().get(0);
                this.time = robotTrajectory.getTimes().get(0);
                break;
            case OUT:
                this.startPoint = robotTrajectory.getPoints().get(robotTrajectory.getPoints().size() - 2);
                this.endPoint = robotTrajectory.getPoints().get(robotTrajectory.getPoints().size() - 1);
                this.speed = robotTrajectory.getSpeeds().get(robotTrajectory.getSpeeds().size() - 1);
                this.time = robotTrajectory.getTimes().get(robotTrajectory.getTimes().size() - 1);
                break;
        }
        this.robotTrajectory = robotTrajectory;
        this.color = robotTrajectory.getConnectionsColor();
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
        return ( endPoint.getX() - startPoint.getX() == 0 ) ? 90 :
        Math.toDegrees(Math.atan((endPoint.getY() - startPoint.getY()) / (endPoint.getX() - startPoint.getX())));
    }

    public double getNormal(){
        double c = startPoint.getX() * endPoint.getY() - endPoint.getX() * startPoint.getY(),
                a = endPoint.getX() - startPoint.getX(),
                b = startPoint.getY() - endPoint.getY();
        return Math.abs(c / Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2)));
    }

    public boolean isPotentialFollowerTo(InOutVector vector){
        double POSSIBLE_ANGLE = 90;

        double n = 15;          //rotation degrees
//        Vec2d vector = new Vec2d(endPoint.getX() - startPoint.getX(), endPoint.getY() - startPoint.getY());
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
                possibleDistance = (this.speed + vector.speed) / 2 * ((vector.time - this.time) / 1000);
//        System.out.println(distance + " " + possibleDistance);
        boolean isInReachableDistance = (distance < possibleDistance * 1.2 && distance > possibleDistance * 0.7) || (
                distance < possibleDistance + 5 && distance > possibleDistance - 5);
        boolean isAzimuthCorrect = Math.abs(this.getAzimuth() - vector.getAzimuth()) < POSSIBLE_ANGLE;
        boolean isInLargeSector = !areClockwise(sectorStart, wayVector) && areClockwise(sectorEnd, wayVector) && isInReachableDistance && isAzimuthCorrect;

//        boolean isInSmallSector;
        return isInLargeSector ;
    }

    private boolean areClockwise(Vec2d v1, Vec2d v2){
        return -v1.x * v2.y + v1.y * v2.x <= 0;
    }

//    public boolean isBehind(InOutVector vector){
//        return (vector.getEndPoint().getX() - startPoint.getX()) / (endPoint.getX() - startPoint.getX()) > 0;
//    }

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
