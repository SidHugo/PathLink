package ru.mcst.RobotGroup.PathsLinking;

import com.sun.javafx.geom.Vec2d;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.Arc2D;
import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by bocharov_n on 14.10.15.
 */
class Camera {

    private int x, y, azimuth, r, angle;
    private Arc2D arc;
    private double accuracy = 1.0;
    private boolean isExist;
    private Tracker tracker;

    public Camera(int x, int y, int azimuth, int r, int angle){
        this.x = x;
        this.y = y;
        this.azimuth = azimuth;
        this.r = r;
        this.angle = angle;
        arc = null;
        redrawFOV();
        isExist = true;
        tracker = null;

    }

    public void redrawFOV(){
        arc = new Arc2D.Double(0.0, 0.5, r, r, 0.0, 60.0, Arc2D.CHORD);         //TODO: is this string necessary?

        arc.setArcByCenter(r, r, r, azimuth - angle / 2, angle, Arc2D.OPEN);


    }

    public boolean isOnCorner(Point2D p){
        boolean isOnArc = Math.abs(Math.sqrt(Math.pow(this.x - p.getX(), 2) + Math.pow(this.y - p.getY(), 2)) - this.r) < this.accuracy;
        boolean isOnLeftLine = Math.abs(p.getY() - this.y -
                Math.tan(Math.toRadians(this.azimuth + this.angle / 2)) * (p.getX() - this.x)) < this.accuracy;
        boolean isOnRightLine = Math.abs(p.getY() - this.y -
                Math.tan(Math.toRadians(this.azimuth - this.angle / 2)) * (p.getX() - this.x)) < this.accuracy;
        return isOnArc || isOnLeftLine || isOnRightLine;
    }

    public boolean isVisible(Point2D point){
        double x = point.getX(), y = point.getY();
        boolean isInCircle = Math.pow(this.getX() - x, 2) + Math.pow(this.getY() - y, 2) <= Math.pow(this.getR(), 2);
        double startX = this.getX() - this.getR() + this.getArc().getStartPoint().getX(),
                startY = this.getY() - this.getR() + this.getArc().getStartPoint().getY(),
                endX = this.getX() - this.getR() + this.getArc().getEndPoint().getX(),
                endY = this.getY() - this.getR() + this.getArc().getEndPoint().getY();
        Point2D startPoint = new Point2D.Double(startX, startY),
                endPoint = new Point2D.Double(endX, endY),
                centerPoint = new Point2D.Float(this.getX(), this.getY());

        Vec2d sectorStart = new Vec2d(startPoint.getX() - centerPoint.getX(), startPoint.getY() - centerPoint.getY()),
                sectorEnd = new Vec2d(endPoint.getX() - centerPoint.getX(), endPoint.getY() - centerPoint.getY()),
                relPoint = new Vec2d(x - centerPoint.getX(), y - centerPoint.getY());
        return isInCircle && !areClockwise(sectorStart, relPoint) && areClockwise(sectorEnd, relPoint);
    }

    private static boolean areClockwise(Vec2d v1, Vec2d v2){
        return -v1.x * v2.y + v1.y * v2.x <= 0;
    }

    public boolean isExist() {
        return isExist;
    }

    public void setExist(boolean exist) {
        isExist = exist;
    }

    public Arc2D getArc() {return arc;}

    public int getX() {
        return x;
    }

    public void setX(int x) {
        this.x = x;
    }

    public int getY() {
        return y;
    }

    public void setY(int y) {
        this.y = y;
    }

    public int getAzimuth() {
        return azimuth;
    }

    public void setAzimuth(int azimuth) {
        this.azimuth = azimuth;
    }

    public int getR() {
        return r;
    }

    public void setR(int r) {
        this.r = r;
    }

    public int getAngle() {
        return angle;
    }

    public void setAngle(int angle) {
        this.angle = angle;
    }

    public Tracker getTracker() {
        return tracker;
    }

    public void setTracker(Tracker tracker) {
        this.tracker = tracker;
    }
}
