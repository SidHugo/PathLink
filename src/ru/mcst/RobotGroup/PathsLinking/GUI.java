package ru.mcst.RobotGroup.PathsLinking;

import ru.mcst.RobotGroup.PathsFinding.Hypervisor;

import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

/**
 * Created by bocharov_n on 22.10.15.
 */
public class GUI extends JFrame{
    private JPanel rootPanel;
    private JButton removeCameraButton;
    private JLabel mouseXLabel;
    private JLabel mouseYLabel;
    private JScrollPane mapScrollPane;
    private JRadioButton selectCameraRadioButton;
    private JRadioButton addCameraRadioButton;
    private JButton linkTrajectoriesButton;
    private JButton clearTrajectoriesButton;
    private JRadioButton selectInOutVectorRadioButton;
    private JSlider cameraAzimuthSlider;
    private JSlider cameraRSlider;
    private JSlider cameraAngleSlider;
    private JLabel cameraAzimuthTextField;
    private JLabel cameraRTextField;
    private JLabel cameraAngleTextField;
    private JRadioButton moveCameraRadioButton;
    private JLabel cameraXLabel;
    private JLabel cameraYLabel;
    private JLabel statusLabel;
    private static MapUnderlay mapPanel;

    private static Camera currentCamera;
    private static InOutVector currentVector;

    public GUI(){
        super();
        long startUITime = System.currentTimeMillis();
        currentCamera   = null;
        createMyComponents();
        startMapListenerDaemon();
        setContentPane(rootPanel);
        setTitle("Paths linking");
        pack();
        setSize(1001, 720);
        setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
        System.out.println("UI load time: " + (System.currentTimeMillis() - startUITime));
    }

    private void startMapListenerDaemon(){
        Thread mapSizeListener = new Thread(){
            @Override
            public void run(){
                while(this.isAlive()) {
                    int[] mapSize = Hypervisor.getMapSize();
                    Image mapImage = Hypervisor.getMapImage();
                    if (mapSize != null && mapSize[0] != mapPanel.getPreferredSize().width &
                            mapSize[1] != mapPanel.getPreferredSize().height) {
                        mapPanel.setPreferredSize(new Dimension(mapSize[0], mapSize[1]));
                        if (mapImage != null){
                            MapUnderlay.changeMapLayer(mapImage);
                        }
                        mapPanel.setSize(mapPanel.getPreferredSize());
                        mapPanel.repaint();


                    }
                    try {
                        sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        };
        mapSizeListener.setDaemon(true);
        mapSizeListener.start();
    }

    private void createUIComponents() {
        mapPanel = new MapUnderlay(this);
        mapScrollPane=new JScrollPane(mapPanel);
    }

    private void createMyComponents(){

        mapPanel.setLayout(new BorderLayout());

        ButtonGroup mapUnderlayToolsGroup = new ButtonGroup();
        mapUnderlayToolsGroup.add(selectCameraRadioButton);
        mapUnderlayToolsGroup.add(addCameraRadioButton);
        mapUnderlayToolsGroup.add(selectInOutVectorRadioButton);
        mapUnderlayToolsGroup.add(moveCameraRadioButton);

        removeCameraButton.setEnabled(false);

        selectCameraRadioButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                mapPanel.setSelectedTool(MapUnderlay.SELECT_CAMERA_TOOL);
            }
        });
        addCameraRadioButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                mapPanel.setSelectedTool(MapUnderlay.ADD_CAMERA_TOOL);
            }
        });
        selectInOutVectorRadioButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                mapPanel.setSelectedTool(MapUnderlay.SELECT_INOUT_VECTOR);
            }
        });
        moveCameraRadioButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                mapPanel.setSelectedTool(MapUnderlay.MOVE_CAMERA);
            }
        });

        cameraAzimuthSlider.addChangeListener(new ChangeListener() {
            @Override
            public void stateChanged(ChangeEvent e) {
                currentCamera.setAzimuth(cameraAzimuthSlider.getValue());
                currentCamera.redrawFOV();
                mapPanel.repaint();
                cameraAzimuthTextField.setText("Camera azimuth:" + cameraAzimuthSlider.getValue());
            }
        });
        cameraAngleSlider.addChangeListener(new ChangeListener() {
            @Override
            public void stateChanged(ChangeEvent e) {
                currentCamera.setAngle(cameraAngleSlider.getValue());
                currentCamera.redrawFOV();
                mapPanel.repaint();
                cameraAngleTextField.setText("Camera angle of view: " + cameraAngleSlider.getValue());
            }
        });
        cameraRSlider.addChangeListener(new ChangeListener() {
            @Override
            public void stateChanged(ChangeEvent e) {
                currentCamera.setR(cameraRSlider.getValue());
                currentCamera.redrawFOV();
                mapPanel.repaint();
                cameraRTextField.setText("Camera radius: " + cameraRSlider.getValue());
            }
        });
        removeCameraButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                TrackingSystem.removeCamera(currentCamera);
                for(int i = 0; i < TrackingSystem.getCameraList().size(); i++){
                    TrackingSystem.getCameraList().get(i).setIndex(i);
                }
                currentCamera.setExist(false);

                currentCamera = null;
                cameraXLabel.setText("X: ");
                cameraYLabel.setText("Y: ");
                cameraAzimuthSlider.setEnabled(false);
//                cameraAzimuthSlider.setValue(0);
                cameraAzimuthTextField.setText("Camera azimuth:");
                cameraAngleSlider.setEnabled(false);
//                cameraAngleSlider.setValue(0);
                cameraAngleTextField.setText("Camera angle of view:");
                cameraRSlider.setEnabled(false);
//                cameraRSlider.setValue(20);
                cameraRTextField.setText("Camera radius:");
                mapPanel.repaint();

                removeCameraButton.setEnabled(false);
            }
        });

        clearTrajectoriesButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                for(Camera camera:TrackingSystem.getCameraList()){
//                    camera.getTracker().clear();
                    camera.getTracker().setMarkForClear(true);
                }
                setCurrentVector(null);
                mapPanel.setCurrentVector(null);
                TrackingSystem.getTrajectoriesList().clear();
                TrackingSystem.getInOutVectorsList().clear();
                mapPanel.clearTrajectoriesLayer();
                mapPanel.clearLinksLayer();
            }
        });

        linkTrajectoriesButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                long startTime = System.currentTimeMillis();
                TrackingSystem.linkTrajectories();
                System.out.println("Trajectories link time(ms): " + (System.currentTimeMillis() - startTime));
            }
        });
    }



    public void updateStatus(){
        String status = "<html>";
        for(Camera curCamera:TrackingSystem.getCameraList()){
            status += "Camera " + TrackingSystem.getCameraList().indexOf(curCamera) + " see " + curCamera.getTracker().getVisibleRobotsCount() +
                    " robots<br>";
        }
        status += "</html>";
        statusLabel.setText(status);
    }

    public void inOutVectorNotification(InOutVector vector){
        String message = (vector.getOrientation() == InOutVector.IN ? "In " : "Out ") +

                "vector: " + TrackingSystem.getInOutVectorsList().indexOf(vector) + System.lineSeparator() +
                "x: " + vector.getX() + System.lineSeparator() +
                "y: " + vector.getY() + System.lineSeparator() +
                "azimuth: " + vector.getAzimuth() + System.lineSeparator() +
                "speed: " + vector.getSpeed() + System.lineSeparator() +
                "acceleration: " + vector.getAcceleration() + System.lineSeparator()  +
                "start time: " + vector.startTime + System.lineSeparator() +
                "end time: " + vector.endTime + System.lineSeparator() +
                "delta time: " + (vector.endTime - vector.startTime) + System.lineSeparator() +
                "connections: " + (vector.getOrientation() == InOutVector.IN ? vector.getPrev().size() : vector.getNext().size());
//        if(vector.getOrientation() == InOutVector.OUT)
//            for(InOutVector inOutVector:TrackingSystem.getInOutVectorsList()){
//                if (inOutVector.getOrientation() == InOutVector.IN){
//                    System.out.println("vector " + TrackingSystem.getInOutVectorsList().indexOf(inOutVector));
//                    vector.isPotentialFollowerTo(inOutVector);
//                }
//            }
//        JOptionPane.showMessageDialog(mapPanel, message);
        System.out.println(message);
    }

    public void setCurrentCamera(Camera camera) {
        GUI.currentCamera = camera;

        cameraXLabel.setText("X: " + camera.getX());
        cameraYLabel.setText("Y: " + camera.getY());
        removeCameraButton.setEnabled(true);

        cameraAzimuthSlider.setEnabled(true);
        cameraAngleSlider.setEnabled(true);
        cameraRSlider.setEnabled(true);
        cameraAzimuthSlider.setValue(camera.getAzimuth());
        cameraAngleSlider.setValue(camera.getAngle());
        cameraRSlider.setValue(camera.getR());
        cameraAzimuthTextField.setText("Camera azimuth: " + cameraAzimuthSlider.getValue());
        cameraAngleTextField.setText("Camera angle of view: " + cameraAngleSlider.getValue());
        cameraRTextField.setText("Camera radius: " + cameraRSlider.getValue());

    }

    /*public void drawLine(StraightLine line){
        double a = line.getA();
        double r = line.getR();

        Point2D point1 = new Point2D.Double(0, r / Math.sin(Math.toRadians(a))),
                point2 = new Point2D.Double(r / Math.cos(Math.toRadians(a)), 0),
                point3 = new Point2D.Double(mapPanel.getWidth(), (r - mapPanel.getWidth() *
                        Math.cos(Math.toRadians(a))) / Math.sin(Math.toRadians(a))),
                point4 = new Point2D.Double((r - mapPanel.getHeight() * Math.sin(Math.toRadians(a))) /
                        Math.cos(Math.toRadians(a)), mapPanel.getHeight());
        Point2D firstPoint = null, secondPoint = null;
        if(point1.getY() >= 0) firstPoint = point1;
        else firstPoint = point2;
        secondPoint = point4;
        if(point3.getX() >= 0 && point3.getY() >= 0) secondPoint = point3;
        if(point2.getX() >= 0 && point2.getY() >= 0) secondPoint = point2;
        BufferedImage lineImage = new BufferedImage(mapPanel.getWidth(), mapPanel.getHeight(), BufferedImage.TYPE_INT_ARGB);
        Graphics2D g2d = lineImage.createGraphics();
        g2d.setComposite(AlphaComposite.Clear);
        g2d.fillRect(0, 0, lineImage.getWidth(), lineImage.getHeight());
        g2d.setComposite(AlphaComposite.Src);
        g2d.setColor(Color.RED);
        g2d.drawLine((int) firstPoint.getX(), (int) firstPoint.getY(), (int) secondPoint.getX(), (int) secondPoint.getY());
        g2d.dispose();
        JLabel lineLabel = new JLabel(new ImageIcon(lineImage));
        mapPanel.add(lineLabel);
        Insets insets = mapPanel.getInsets();
        Dimension size = lineLabel.getPreferredSize();
        lineLabel.setBounds(insets.left, insets.top, size.width, size.height);
    }*/

    public JLabel getMouseXLabel() {
        return mouseXLabel;
    }

    public JLabel getMouseYLabel() {
        return mouseYLabel;
    }

    public JLabel getCameraXLabel() {
        return cameraXLabel;
    }

    public JLabel getCameraYLabel() {
        return cameraYLabel;
    }

    public static MapUnderlay getMapPanel() {
        return mapPanel;
    }

    public void setCurrentVector(InOutVector currentVector) {
        GUI.currentVector = currentVector;
    }

}
