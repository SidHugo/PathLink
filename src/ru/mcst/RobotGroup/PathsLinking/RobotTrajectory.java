package ru.mcst.RobotGroup.PathsLinking;

import java.awt.*;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Random;


class RobotTrajectory {
    private ArrayList<Point2D> points;
    private int direction;             //0 - nothing; 1 - in && out; 2 - only in; 3 - only out
    private Color connectionsColor;
    private ArrayList<RobotTrajectory> connectedTrajectories;
    private InOutVector inVector, outVector;

    public RobotTrajectory(){
        points = new ArrayList<Point2D>();
        direction = 0;
        generateNewColor();
        inVector = null;
        outVector = null;
        connectedTrajectories = new ArrayList<RobotTrajectory>();
    }

    public RobotTrajectory(ArrayList<Point2D> points, int directions){
        points = new ArrayList<Point2D>();
        for(Point2D point2D:points){
            this.points.add(point2D);
        }
        this.direction = directions;
        generateNewColor();
        inVector = null;
        outVector = null;
        connectedTrajectories = new ArrayList<RobotTrajectory>();
    }

    public RobotTrajectory(RobotTrajectory robotTrajectory){
        points = new ArrayList<Point2D>();
        for(Point2D point2D:robotTrajectory.getPoints()){
             this.points.add(point2D);
        }
        this.direction = robotTrajectory.getDirection();
        generateNewColor();
        inVector = null;
        outVector = null;
        connectedTrajectories = new ArrayList<RobotTrajectory>();
    }

    public void generateNewColor(){
        Random random = new Random();
        float r = random.nextFloat();
        float g = random.nextFloat();
        float b = random.nextFloat();
        connectionsColor = new Color(r, g, b);
    }

    public ArrayList<Point2D> getPoints() {
        return points;
    }

    public int getDirection() {
        return direction;
    }

    public void setDirection(int direction) {
        this.direction = direction;
    }

    public Color getConnectionsColor() {
        return connectionsColor;
    }

    public void setConnectionsColor(Color connectionsColor) {
        this.connectionsColor = connectionsColor;
    }

    public InOutVector getInVector() {
        return inVector;
    }

    public void setInVector(InOutVector inVector) {
        this.inVector = inVector;
    }

    public InOutVector getOutVector() {
        return outVector;
    }

    public void setOutVector(InOutVector outVector) {
        this.outVector = outVector;
    }

    public ArrayList<RobotTrajectory> getConnectedTrajectories() {
        return connectedTrajectories;
    }

    public void setConnectedTrajectories(ArrayList<RobotTrajectory> connectedTrajectories) {
        this.connectedTrajectories = connectedTrajectories;
    }


}