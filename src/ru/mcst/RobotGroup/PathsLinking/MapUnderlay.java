package ru.mcst.RobotGroup.PathsLinking;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.geom.Arc2D;
import java.awt.image.BufferedImage;

/**
 * Created by bocharov_n on 02.03.16.
 */
class MapUnderlay extends JPanel implements MouseListener, MouseMotionListener{
    public static final int SELECT_CAMERA_TOOL = 0;
    public static final int ADD_CAMERA_TOOL = 1;
    public static final int SELECT_INOUT_VECTOR = 2;
    public static final int MOVE_CAMERA = 3;

    private GUI parentGUI;

    private static BufferedImage mapLayer, trajectoriesLayer, camerasLayer, linksLayer;
    private int selectedTool;
    private Camera currentCamera;
    private InOutVector currentVector;
    private final int cameraSize = 16,
                        vectorCircleSize = 20;



    public MapUnderlay(GUI gui){
        super();
        mapLayer = null;
        trajectoriesLayer = null;
        camerasLayer = null;
        linksLayer = null;
        currentCamera = null;
        currentVector = null;
        addMouseListener(this);
        addMouseMotionListener(this);
        parentGUI = gui;
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        if(mapLayer != null) {
            g2d.drawImage(mapLayer, 0, 0, null);
            this.setPreferredSize(new Dimension(mapLayer.getWidth(), mapLayer.getHeight()));
        }
        if(camerasLayer != null){
            updateCamerasLayer();
            g2d.drawImage(camerasLayer, 0, 0, null);
        }
        if(trajectoriesLayer != null){
            g2d.drawImage(trajectoriesLayer, 0, 0, null);
        }
        if(linksLayer != null){
            updateLinksLayer();
            g2d.drawImage(linksLayer, 0, 0, null);
        }
        g2d.dispose();
        parentGUI.updateStatus();
    }

    private void updateLinksLayer(){
        Graphics2D g2d = linksLayer.createGraphics();
        g2d.setComposite(AlphaComposite.Clear);
        g2d.fillRect(0, 0, linksLayer.getWidth(), linksLayer.getHeight());
        g2d.setComposite(AlphaComposite.Src);
        if(currentVector != null) {
            for (InOutVector vector : TrackingSystem.getInOutVectorsList()) {
                vector.drawVector(g2d, vector == currentVector, true);
            }
            for (InOutVector vector : currentVector.getOrientation() == 0 ? currentVector.getPrev() : currentVector.getNext()) {
                vector.drawVector(g2d, true, false);
            }
        }
        else{
            for (InOutVector vector : TrackingSystem.getInOutVectorsList()) {
                vector.drawVector(g2d, false, false);
            }
        }
        g2d.dispose();
    }

    private void updateCamerasLayer(){
        Graphics2D g2d = camerasLayer.createGraphics();
        g2d.setComposite(AlphaComposite.Clear);
        g2d.fillRect(0, 0, camerasLayer.getWidth(), camerasLayer.getHeight());
        g2d.setComposite(AlphaComposite.Src);
        g2d.setColor(Color.WHITE);
        for(Camera camera:TrackingSystem.getCameraList()){
            if(camera != currentCamera){
                g2d.setColor(Color.WHITE);
            }
            else{
                g2d.setColor(Color.RED);
            }
            g2d.fillOval(camera.getX() - cameraSize / 2, camera.getY() - cameraSize / 2, cameraSize, cameraSize);
            g2d.setColor(Color.BLACK);
            g2d.drawOval(camera.getX() - cameraSize / 2, camera.getY() - cameraSize / 2, cameraSize, cameraSize);

            //Draw FOV
            int     r = camera.getR(),
                    azimuth = camera.getAzimuth(),
                    angle = camera.getAngle();
            Arc2D arc = new Arc2D.Double(0.0, 0.5, r, r, 0.0, 60.0, Arc2D.CHORD);

            arc.setArcByCenter(camera.getX(), camera.getY(), r, azimuth - angle / 2, angle, Arc2D.OPEN);

            g2d.setComposite(AlphaComposite.Src);
            g2d.draw(arc);
            if(angle < 360) {
                g2d.drawLine(camera.getX(), camera.getY(), new Double(arc.getStartPoint().getX()).intValue(), new Double(arc.getStartPoint().getY()).intValue());
                g2d.drawLine(camera.getX(), camera.getY(), new Double(arc.getEndPoint().getX()).intValue(), new Double(arc.getEndPoint().getY()).intValue());
            }
        }
        g2d.dispose();
    }

    public static void changeMapLayer(Image image){
        mapLayer = new BufferedImage(image.getWidth(null),image.getHeight(null), BufferedImage.TYPE_INT_ARGB);
        trajectoriesLayer = new BufferedImage(image.getWidth(null),image.getHeight(null), BufferedImage.TYPE_INT_ARGB);
        camerasLayer = new BufferedImage(image.getWidth(null),image.getHeight(null), BufferedImage.TYPE_INT_ARGB);
        linksLayer = new BufferedImage(image.getWidth(null),image.getHeight(null), BufferedImage.TYPE_INT_ARGB);
        Graphics2D g2d = mapLayer.createGraphics();
        g2d.drawImage(image, 0, 0, null);
        g2d.dispose();

    }

    public static void changeTrajectoriesLayer(BufferedImage trajectories) {
        Graphics2D g2d = trajectoriesLayer.createGraphics();
        g2d.drawImage(trajectories, 0, 0, null);
        g2d.dispose();
    }

    public static BufferedImage getMapLayer() {
        return mapLayer;
    }

    public static BufferedImage getTrajectoriesLayer() {
        return trajectoriesLayer;
    }

    public void clearTrajectoriesLayer(){
        Graphics2D g2d = trajectoriesLayer.createGraphics();
        g2d.setComposite(AlphaComposite.Clear);
        g2d.fillRect(0, 0, trajectoriesLayer.getWidth(), trajectoriesLayer.getHeight());
        g2d.dispose();
    }

    public void clearLinksLayer(){
        Graphics2D g2d = linksLayer.createGraphics();
        g2d.setComposite(AlphaComposite.Clear);
        g2d.fillRect(0, 0, linksLayer.getWidth(), linksLayer.getHeight());
        g2d.dispose();
    }

    public void setSelectedTool(int selectedTool) {
        this.selectedTool = selectedTool;
    }

    public void setCurrentVector(InOutVector currentVector) {
        this.currentVector = currentVector;
    }

    private void moveCamera(int x, int y) {
        currentCamera.setX(x);
        currentCamera.setY(y);
        currentCamera.redrawFOV();
        parentGUI.getCameraXLabel().setText("X: " + x);
        parentGUI.getCameraYLabel().setText("Y: " + y);
    }

    @Override
    public void mouseClicked(MouseEvent e) {
        long startTime = System.currentTimeMillis();
        double minDistance;
        switch(selectedTool){
            case SELECT_CAMERA_TOOL:
                Camera currentCamera = null;
                minDistance = cameraSize / 2 + 1;
                for(Camera camera:TrackingSystem.getCameraList()){
                    double distance = Math.sqrt(Math.pow(camera.getX() - e.getX(), 2) + Math.pow(camera.getY() - e.getY(), 2));
                    if(distance < minDistance){
                        minDistance = distance;
                        currentCamera = camera;
                    }
                }

                if (currentCamera != null) {
                    System.out.println("Camera " + currentCamera.getIndex() + " at " + currentCamera.getX() + " " + currentCamera.getY() + " selected");
                    this.currentCamera = currentCamera;
                    parentGUI.setCurrentCamera(currentCamera);
                }
                break;
            case ADD_CAMERA_TOOL:
                System.out.println("Add new camera");
                System.out.println(e.getX()+" "+ e.getY());
                Camera newCamera = new Camera(e.getX(), e.getY(), 90, 120, 90);
                TrackingSystem.addCamera(newCamera);
                Tracker tracker = new Tracker(newCamera);
                newCamera.setTracker(tracker);
                tracker.setDaemon(true);
                tracker.start();
                break;
            case SELECT_INOUT_VECTOR:
                InOutVector currentVector = null;
                minDistance = vectorCircleSize / 2 + 1;
                for(InOutVector inOutVector:TrackingSystem.getInOutVectorsList()){
                    double distance = Math.sqrt(Math.pow(inOutVector.getX() - e.getX(), 2) + Math.pow(inOutVector.getY() - e.getY(), 2));
                    if(distance < minDistance){
                        minDistance = distance;
                        currentVector = inOutVector;
                    }
                }
                if(currentVector != null){
                    System.out.println("Vector " + (int)currentVector.getX() + " " + (int)currentVector.getY() + " selected");
                    this.currentVector = currentVector;
                    //set curVector and update fields
                    parentGUI.inOutVectorNotification(currentVector);
                    parentGUI.setCurrentVector(currentVector);
                }
                break;
            case MOVE_CAMERA:
                moveCamera(e.getX(), e.getY());
                break;
        }
        repaint();
        System.out.println("Click event handling(ms): " + (System.currentTimeMillis() - startTime));
    }

    @Override
    public void mousePressed(MouseEvent e) {

    }

    @Override
    public void mouseReleased(MouseEvent e) {

    }

    @Override
    public void mouseEntered(MouseEvent e) {

    }

    @Override
    public void mouseExited(MouseEvent e) {

    }


    @Override
    public void mouseDragged(MouseEvent e) {
        switch (selectedTool){
            case MOVE_CAMERA:
                moveCamera(e.getX(), e.getY());
                break;
        }
        repaint();
    }

    @Override
    public void mouseMoved(MouseEvent e) {
        parentGUI.getMouseXLabel().setText("x:" + e.getX());
        parentGUI.getMouseYLabel().setText("y:" + e.getY());
        parentGUI.repaint();
    }
}
