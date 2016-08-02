package ru.mcst.RobotGroup.PathsFinding;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

class Surface extends JPanel implements MouseListener, MouseMotionListener, KeyListener{
	public static final int STAGE_PLACE_ROBOT=6;
	public static final int STAGE_SET_FINISH=7;
	public static final int STAGE_SET_RECT_WEIGHT=8;
	public static final int STAGE_SET_FINISH_ALL=9;

	// ������ �������� (��� ����������� ��������� ��������
	// ��� ������ ��������)
	private int stage;
    // �����
	private Robot robot;
	private boolean showMap=true;
	// ���������� �� ����� ������������
    private boolean showPassability=true;
    // ���������� �� ����� ����������
    private boolean showReality=false;
    // ����������� ������
	private Image robotImage;
	private Image robotImageOriginal;
	private Image robotImageSelected;
	private Image robotImageSelectedOriginal;
	// ������� ����
    private int nodeDiameter=10;
    // ���������� �� ����
    private boolean showNodes=false;
    // ����, ������������ ����
    private BufferedImage screen;

    private Image finishDirectionImage;
    //private Node finish=new Node(Robot.ROBOT_NOWHERE_X, Robot.ROBOT_NOWHERE_Y, 0);
    
    private int rectWeight=0;

	private List<Robot> robots = new ArrayList<Robot>();
	private Hypervisor hypervisor=new Hypervisor(robots);

	private boolean robotsRunning=false;
	private boolean logging=false;


	public Surface() {
        addMouseListener(this);
        addMouseMotionListener(this);
        addKeyListener(this);
        stage=0;
        robot=new Robot();
		robots.add(robot);
		//robots.add(new Robot());
        robotImage=new ImageIcon(getClass().getResource("/robotR2D2.png")).getImage();
		robotImageOriginal=new ImageIcon(getClass().getResource("/robotR2D2.png")).getImage();
		robotImageSelected=new ImageIcon(getClass().getResource("/robotR2D2Selected.png")).getImage();
		robotImageSelectedOriginal=new ImageIcon(getClass().getResource("/robotR2D2Selected.png")).getImage();
        finishDirectionImage=new ImageIcon(getClass().getResource("/arrow.png")).getImage();
        
        screen=null;
	}
	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
        doDrawing(g);
	}
	// ������� ���������
	private void doDrawing(Graphics g) {
		Graphics2D g2d = (Graphics2D) g;
		if(showMap)
			g2d.drawImage(robot.getMap().getImage(), 0, 0, null);
		if((showMap?1:0)+(showPassability?1:0)+(showReality?1:0)>1) {
			g2d.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, 0.5f));
		}
		if(showPassability) {
			g2d.drawImage(robot.getMap().getPassabilityMap(), 0, 0, null);
		}
		if(showReality) {
			g2d.drawImage(robot.getMap().getRealityMap(), 0, 0, null);
		}
		g2d.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, 1));		
		
		if(showNodes) {
			if(robot.getMap().getIsChanged()){
				robot.getMap().setIsChanged(false);
				refreshScreen();				
			}
			g2d.drawImage(screen, 0, 0, null);
		}

		g2d.setStroke(new BasicStroke(2));
		g2d.setColor(Color.cyan);
//		if(robot.hasPath() && !robot.isMapChangedSignal() && robot.getSpeed()!=0)
//		{
//			Node from=robot.getSearchAlgorithm().getPath().get(0);
//			for(int i=1; i<robot.getSearchAlgorithm().getPath().size(); ++i)
//			{
//				Node to=robot.getSearchAlgorithm().getPath().get(i);
//				Link link=from.getLinkByChild(to);
//				if(link!=null) {
//					for(Segment s: link.getSegments()) {
//						if(s.getIsStraightLine()){
//							g2d.drawLine((int)s.getOriginX(), (int)s.getOriginY(), (int)(s.getOriginX()+s.getLength()*Math.cos(s.getStartAngle())), (int)(s.getOriginY()-s.getLength()*Math.sin(s.getStartAngle())));
//						} else {
//							int degreesTotal=(int)Math.toDegrees(s.getRadiansTotal());
//							if(s.getIsClockwise())
//								degreesTotal*=-1;
//							g2d.drawArc((int)(s.getOriginX()-s.getRadius()), (int)(s.getOriginY()-s.getRadius()), (int)s.getRadius()*2, (int)s.getRadius()*2, (int)Math.toDegrees(s.getStartAngle()), degreesTotal);
//						}
//					}
//				}
//				from=to;
//			}
//		}
		for(Robot r:robots) {
			if (r.hasPath() && !r.isMapChangedSignal() && r.getSpeed() != 0) {
				Node from = r.getSearchAlgorithm().getPath().get(0);
				for (int i = 1; i < r.getSearchAlgorithm().getPath().size(); ++i) {
					Node to = r.getSearchAlgorithm().getPath().get(i);
					Link link = from.getLinkByChild(to);
					if (link != null) {
						for (Segment s : link.getSegments()) {
							if (s.getIsStraightLine()) {
								g2d.drawLine((int) s.getOriginX(), (int) s.getOriginY(), (int) (s.getOriginX() + s.getLength() * Math.cos(s.getStartAngle())), (int) (s.getOriginY() - s.getLength() * Math.sin(s.getStartAngle())));
							} else {
								int degreesTotal = (int) Math.toDegrees(s.getRadiansTotal());
								if (s.getIsClockwise())
									degreesTotal *= -1;
								g2d.drawArc((int) (s.getOriginX() - s.getRadius()), (int) (s.getOriginY() - s.getRadius()), (int) s.getRadius() * 2, (int) s.getRadius() * 2, (int) Math.toDegrees(s.getStartAngle()), degreesTotal);
							}
						}
					}
					from = to;
				}
			}
		}
//		AffineTransform at = new AffineTransform();
//		g2d.setColor(Color.green);
//		g2d.drawOval((int) finish.getX() - 5, (int) finish.getY() - 5, 10, 10);
//		at.translate(finish.getX(), finish.getY() - 5);
//		at.rotate((-1) * finish.getDirection(), 0, finishDirectionImage.getHeight(null) / 2);
//		g2d.drawImage(finishDirectionImage, at, null);

//		at = new AffineTransform();
//		at.translate((int)robot.getX()-robotImage.getWidth(null)/2, (int)robot.getY()-robotImage.getHeight(null)/2);
//		at.rotate((-1)*robot.getAzimuth(), robotImage.getWidth(null)/2, robotImage.getHeight(null)/2);
//		g2d.drawImage(robotImage, at, null);

		g2d.setBackground(new Color(0,0,0,1));
		for(Robot r:robots) {
			AffineTransform at = new AffineTransform();
			if(r==robot) {
				g2d.setColor(Color.green);
			} else {
				g2d.setColor(Color.orange);
			}
			Node robotFinish = r.getFinish();
			g2d.drawOval((int) robotFinish.getX() - 5, (int) robotFinish.getY() - 5, 10, 10);
			g2d.drawImage(rotate(finishDirectionImage,
							(-1)*robotFinish.getDirection()),
					(int)robotFinish.getX()-finishDirectionImage.getWidth(null)/2,
					(int)robotFinish.getY()-finishDirectionImage.getWidth(null)/2,
					null);

			if(r!=robot) {
				g2d.drawImage(rotate(robotImage, (-1) * r.getAzimuth()),
						(int)r.getX()- robotImage.getWidth(null) / 2,
						(int)r.getY()- robotImage.getHeight(null) / 2,
						null);
			} else {
				g2d.drawImage(rotate(robotImageSelected,
								(-1) * r.getAzimuth()),
						(int)r.getX()- robotImage.getHeight(null) / 2,
						(int)r.getY()- robotImage.getHeight(null) / 2,
						null);
			}
		}
	}

	private static Image rotate(Image image, double radians) {
		int width = image.getWidth(null);
		int height = image.getHeight(null);
		int size = Math.max(width, height);
		BufferedImage temp = new BufferedImage(size, size, BufferedImage.TYPE_INT_ARGB);
		Graphics2D g2 = temp.createGraphics();
		g2.rotate(radians, size / 2, size / 2);
		g2.setBackground(new Color(1, 0, 0, 1));
		g2.drawImage(image, 0, 0, null);
		g2.dispose();
		return temp;
	}

	private static Image othersRotate(Image image, double radians, int xCenter, int yCenter) {
		int width = image.getWidth(null)*2;
		int height = image.getHeight(null)*2;
		int size=Math.max(width, height);
		//BufferedImage temp = new BufferedImage(height, width, BufferedImage.TYPE_INT_ARGB);
		BufferedImage temp = new BufferedImage(size, size, BufferedImage.TYPE_INT_ARGB);
		Graphics2D g2 = temp.createGraphics();
		//g2.rotate(radians, height / 2, height / 2);
		g2.rotate(radians, xCenter, yCenter);
		g2.setBackground(new Color(1, 0, 0, 1));
		g2.drawImage(image, 0, 0, null);
		g2.dispose();
		return temp;
	}

	// ������� ���������� ��������� �����
	public void refreshScreen() {
		if(robot.getMap().getImage()!=null)
			screen=new BufferedImage(robot.getMap().getImage().getWidth(null), robot.getMap().getImage().getHeight(null), BufferedImage.TYPE_INT_ARGB);
		else
			screen=new BufferedImage(getWidth(), getHeight(), BufferedImage.TYPE_INT_ARGB);
		Graphics2D g2d = screen.createGraphics();
		
		g2d.setColor(Color.blue);
		g2d.setStroke(new BasicStroke(1));
        
        List<Node> nodes=robot.getMap().getNodesInList();
        for(Node n:nodes)
		{
			g2d.drawOval((int)n.getX()-5, (int)n.getY()-5, nodeDiameter, nodeDiameter);
			for(int i=0; i<n.getLinks().size(); ++i) {
				Link link=n.getLinks().get(i);
				g2d.drawLine((int)n.getX(), (int)n.getY(), (int)link.getChild().getX(), (int)link.getChild().getY());
			}
		}
		
	}
	public void mouseClicked(MouseEvent arg0) {
		if(robotsRunning)
			return;
		if(arg0.getButton()==MouseEvent.BUTTON1) {
			switch (stage) {				
			case STAGE_PLACE_ROBOT:
				placeRobot(arg0.getX(), arg0.getY());
				break;		
			case STAGE_SET_FINISH:
				setFinish(arg0.getX(), arg0.getY());
				break;
			case STAGE_SET_RECT_WEIGHT:
				changeRectangle(arg0.getX(), arg0.getY());
				break;
			case STAGE_SET_FINISH_ALL:
				setFinishAll(arg0.getX(), arg0.getY());
				break;
			default:
				break;
			}
		}	
	}
	public void mouseEntered(MouseEvent arg0) {
		// TODO Auto-generated method stub
		
	}
	public void mouseExited(MouseEvent arg0) {
		// TODO Auto-generated method stub
		
	}
	public void mousePressed(MouseEvent arg0) {
		requestFocus();
	}
	public void mouseReleased(MouseEvent arg0) {
		// TODO Auto-generated method stub
	}
	@Override
	public void mouseDragged(MouseEvent arg0) {
		if(robotsRunning)
			return;
		if(SwingUtilities.isLeftMouseButton(arg0)) {
			switch (stage) {				
				case STAGE_PLACE_ROBOT:
					placeRobot(arg0.getX(), arg0.getY());
					break;		
				case STAGE_SET_FINISH:
					setFinish(arg0.getX(), arg0.getY());
					break;
				case STAGE_SET_RECT_WEIGHT:
					changeRectangle(arg0.getX(), arg0.getY());
					break;
				case STAGE_SET_FINISH_ALL:
					setFinishAll(arg0.getX(), arg0.getY());
					break;
				default:
					break;					
			}
		}
	}
	@Override
	public void mouseMoved(MouseEvent arg0) {
		// TODO Auto-generated method stub		
	}
	private void placeRobot(int x, int y) {
		int scale=robot.getMap().getScale();		
		if(robot.getMap().getImage()!=null) {
			if(x<scale/2)
				x=scale/2;
			else if(x>robot.getMap().getWidth()-scale/2)
				x=robot.getMap().getWidth()-scale/2;
			if(y<scale/2)
				y=scale/2;
			else if(y>robot.getMap().getHeight()-scale/2)
				y=robot.getMap().getHeight()-scale/2;
		}
		// check if robot can stand there
		if(Link.getPointWeight(x, y, robot.getAzimuth(), scale, robot.getMap().getPassabilityArray())==255)
			return;
		robot.setX(x);
		robot.setY(y);
		//Control_Panel.setRobotCoordinates(x, y);
		robot.setMapChangedSignal(true);
		repaint();
	}
	private void setFinish(int x, int y) {
		int scale=robot.getMap().getScale();		
		if(robot.getMap().getImage()!=null) {
			if(x<scale/2)
				x=scale/2;
			else if(x>robot.getMap().getWidth()-scale/2)
				x=robot.getMap().getWidth()-scale/2;
			if(y<scale/2)
				y=scale/2;
			else if(y>robot.getMap().getHeight()-scale/2)
				y=robot.getMap().getHeight()-scale/2;
		}
		Node finish=robot.getFinish();
		// check if robot can stand there
		if(Link.getPointWeight(x, y, finish.getDirection(), scale, robot.getMap().getPassabilityArray())==255)
			return;
		finish.setX(x);
		finish.setY(y);

		if (robot.getSpeed() != 0) {
			//synchronized (robot.getMap().getFinish()) {
			synchronized (robot.getMap().getNodes()) {
				Node n = new Node(finish.getX(), finish.getY(), finish.getDirection());
				n.setIsRobotMade(true);
				robot.getMap().addNode(n);
				robot.getMap().addLinksAroundCell24(n, robot.getRadius(), true);
				//robot.addNodeToDelete(robot.getMap().getFinish());
				robot.addNodeToDelete(robot.getFinish());
				//robot.getMap().setFinish(n);
				robot.setFinish(n);
			}
		} else {
//			//if some robot is moving, then everybody is moving. so this robot needs to move too
//			for(Robot r: robots) {
//				if (r.getSpeed() != 0) {
//					robot.move(/*new Node(r.getX(), r.getY(), r.getAzimuth()), */new Node(finish.getX(), finish.getY(), finish.getDirection()));
//				}
//			}
		}
		robot.setMapChangedSignal(true);
		//Control_Panel.setFinishCoordinates(x, y);
		//robot.setMapChangedSignal(true);
		repaint();
	}
	private void setFinishAll(int x, int y) {
		int scale=robot.getMap().getScale();
		if(robot.getMap().getImage()!=null) {
			if(x<scale/2)
				x=scale/2;
			else if(x>robot.getMap().getWidth()-scale/2)
				x=robot.getMap().getWidth()-scale/2;
			if(y<scale/2)
				y=scale/2;
			else if(y>robot.getMap().getHeight()-scale/2)
				y=robot.getMap().getHeight()-scale/2;
		}
		for(Robot r: robots) {
			Node finish = r.getFinish();
			// check if robot can stand there
			if (Link.getPointWeight(x, y, finish.getDirection(), scale, r.getMap().getPassabilityArray()) == 255)
				return;
			finish.setX(x);
			finish.setY(y);

			if (r.getSpeed() != 0) {
				//synchronized (robot.getMap().getFinish()) {
				synchronized (r.getMap().getNodes()) {
					Node n = new Node(finish.getX(), finish.getY(), finish.getDirection());
					n.setIsRobotMade(true);
					r.getMap().addNode(n);
					r.getMap().addLinksAroundCell24(n, r.getRadius(), true);
					r.addNodeToDelete(r.getFinish());
					r.setFinish(n);
				}
			} else {
//			//if some robot is moving, then everybody is moving. so this robot needs to move too
//			for(Robot r: robots) {
//				if (r.getSpeed() != 0) {
//					robot.move(/*new Node(r.getX(), r.getY(), r.getAzimuth()), */new Node(finish.getX(), finish.getY(), finish.getDirection()));
//				}
//			}
			}
			robot.setMapChangedSignal(true);
		}
		repaint();
	}
	public void render() {
		SwingUtilities.invokeLater(new Runnable() {
			public void run() {
				repaint();  // repaint(), etc. according to changed states
			}
		});
	}
	public void resizeRobot(int size) {
		BufferedImage resized=new BufferedImage(size, size, BufferedImage.TYPE_INT_ARGB);
		Graphics2D g2d=resized.createGraphics();
		g2d.drawImage(robotImageOriginal, 0, 0, size, size, null);
		robotImage=resized;

		resized=new BufferedImage(size, size, BufferedImage.TYPE_INT_ARGB);
		g2d=resized.createGraphics();
		g2d.drawImage(robotImageSelectedOriginal, 0, 0, size, size, null);
		robotImageSelected=resized;
		g2d.dispose();
	}
    public void setScale(int scale) {
//        BufferedImage resized=new BufferedImage(scale, scale, BufferedImage.TYPE_INT_ARGB);
//        Graphics2D g2d=resized.createGraphics();
//        g2d.drawImage(robotImage, 0, 0, scale, scale, null);
//        robotImage=resized;
		resizeRobot(scale);
        robot.getMap().setScale(scale);
//        g2d.dispose();
		repaint();
    }
    public void setStage(int stage) {
    	this.stage=stage;
		repaint();
    }
    public void runRobot() {
    	long time=System.currentTimeMillis();
        long searchTime = robot.move(/*new Node(robot.getX(), robot.getY(), robot.getAzimuth()),*/ /*new Node(finish.getX(), finish.getY(), finish.getDirection())*/);
		JOptionPane.showMessageDialog(this, "Robot stopped after " + (System.currentTimeMillis() - time) + " ms\nFirst AStar search lasted " + searchTime + " ms");
    }
    public void stopRobot() {
        robot.setStopSignal(true);
    }
	public void stopRobots() {
		for(Robot r: robots) {
			r.setStopSignal(true);
		}
	}
    public Robot getRobot() {return robot;}
//    public void removeRobot() {
//        //robot.getMap().setStanding(null);
//		robot.setStanding(null);
//        robot.setX(Robot.ROBOT_NOWHERE_X);
//        robot.setY(Robot.ROBOT_NOWHERE_Y);
//        //Control_Panel.setRobotCoordinates(-1000, -1000);
//        robot.setMapChangedSignal(true);
//    }
	public void removeRobots() {
		for(Robot r: robots) {
			r.setStanding(null);
			r.setX(Robot.ROBOT_NOWHERE_X);
			r.setY(Robot.ROBOT_NOWHERE_Y);
			r.setMapChangedSignal(true);
		}
	}
//    public void removeFinish() {
//		//robot.getMap().setFinish(null);
//		robot.setFinish(null);
//    	finish.setX(-1000);
//    	finish.setY(-1000);
//    	//Control_Panel.setFinishCoordinates(-1000, -1000);
//    	robot.setMapChangedSignal(true);
//    }
	public void removeFinishes() {
		for (Robot r : robots) {
			r.getFinish().setX(Robot.ROBOT_NOWHERE_X);
			r.getFinish().setY(Robot.ROBOT_NOWHERE_X);
			r.setMapChangedSignal(true);
		}
	}
    public void removeAllNodes() {
        robot.getMap().getNodes().clear();
        removeRobots();
        removeFinishes();
        refreshScreen();
        repaint();
    }
    public void setShowMap(boolean showMap) {this.showMap=showMap;}
    public void setShowPassability(boolean showPassability) {this.showPassability=showPassability;}
    public void setShowReality(boolean showReality) {this.showReality=showReality;}
    public void setShowNodes(boolean showNodes) {this.showNodes=showNodes;}
//    public void setRobotSensorsRange(int sensorsRange) {
//		robot.setSensorsRange(sensorsRange);
//	}
	public void setRobotsSensorsRange(int sensorsRange) {
		for(Robot r: robots) {
			r.setSensorsRange(sensorsRange);
		}
	}
    //public void setRobotSpeed(int speed) { robot.setSpeed(speed); }
	public void setRobotsSpeed(int speed) {
		for(Robot r: robots) {
			r.setSpeed(speed);
		}
	}
//    public void setRealityMap(BufferedImage realityMap) {
//		robot.getMap().setRealityMap(realityMap);
//	}
	public void setRealityMaps(BufferedImage realityMap) {
		for(Robot r: robots) {
			r.getMap().setRealityMap(realityMap);
		}
	}
//    public void setMapImages(BufferedImage image) {
//    	robot.getMap().setImage(image);
//    	removePassability();
//    }
	public void setMapImages(BufferedImage image) {
		for(Robot r: robots) {
			r.getMap().setImage(image);
			removePassabilities();
		}
	}
//    public void removePassability() {
//    	robot.getMap().removePassability();
//    	removeRobot();
//    	removeFinish();
//    }
	public void removePassabilities() {
		for(Robot r: robots) {
			r.getMap().removePassability();
		}
		removeRobots();
		removeFinishes();
	}
//    public void setRobotMinSpeed(int minSpeed) {robot.setMinSpeed(minSpeed);}
//    public void setRobotMaxSpeed(int maxSpeed) {robot.setMaxSpeed(maxSpeed);}
	public void setRobotsMinSpeed(int minSpeed) {
		for(Robot r: robots) {
			r.setMinSpeed(minSpeed);
		}
	}
	public void setRobotsMaxSpeed(int maxSpeed) {
		for(Robot r: robots) {
			r.setMaxSpeed(maxSpeed);
		}
	}
    public double getRobotSpeed() {return robot.getSpeed();}
//    public void setRobotAcceleration(int acceleration) {robot.setAcceleration(acceleration);}
//    public void setRobotDeceleration(int deceleration) {robot.setDeceleration(deceleration);}
	public void setRobotsAcceleration(int acceleration) {
		for(Robot r: robots) {
			r.setAcceleration(acceleration);
		}
	}
	public void setRobotsDeceleration(int deceleration) {
		for(Robot r: robots) {
			r.setDeceleration(deceleration);
		}
	}
    public void setRobotAzimuth(double azimuth) {
		if (Link.getPointWeight((int) robot.getX(), (int) robot.getY(), azimuth, robot.getMap().getScale(), robot.getMap().getPassabilityArray())==255)
    		return;
    	robot.setAzimuth(azimuth);
    	robot.setMapChangedSignal(true);
    	repaint();
    }
//    public void setFinishDirection(double direction) {
//    	if(Link.getPointWeight((int)finish.getX(), (int)finish.getY(), direction, robot.getMap().getScale(), robot.getMap().getPassabilityArray())==255)
//    		return;
//    	finish.setDirection(direction);
//    	// synchronized with function cleanup. when addNode called here, node can be removed
//    	// by robot.cleanup before this node became finish. so robot will stop.
//    	//if(robot.getMap().getFinish()!=null) {
//		if(robot.getFinish()!=null) {
//    		synchronized (robot.getMap().getNodes()) {
//				if(robot.getSpeed()!=0) {
//		    		Node n=new Node(finish.getX(), finish.getY(), finish.getDirection());
//		    		n.setIsRobotMade(true);
//		    		robot.getMap().addNode(n);
//		    		robot.getMap().addLinksAroundCell24(n, robot.getRadius(), true);
////		    		robot.addNodeToDelete(robot.getMap().getFinish());
////		    		robot.getMap().setFinish(n);
//					robot.addNodeToDelete(robot.getFinish());
//					robot.setFinish(n);
//		    	}
//	    	}
//    	}
//    	robot.setMapChangedSignal(true);
//    	repaint();
//    }
	public void setFinishDirection(double direction) {
		if(stage!=STAGE_SET_FINISH_ALL) {
			Node finish = robot.getFinish();
			if (Link.getPointWeight((int) finish.getX(), (int) finish.getY(), direction, robot.getMap().getScale(), robot.getMap().getPassabilityArray()) == 255)
				return;
			finish.setDirection(direction);
			robot.setMapChangedSignal(true);
		} else {
			for(Robot r: robots) {
				Node finish = r.getFinish();
				if (Link.getPointWeight((int) finish.getX(), (int) finish.getY(), direction, r.getMap().getScale(), r.getMap().getPassabilityArray()) == 255)
					return;
				finish.setDirection(direction);
				r.setMapChangedSignal(true);
			}
		}
		repaint();
	}
    //public double getFinishDirection() {return finish.getDirection();}
	public double getFinishDirection() {return robot.getFinish().getDirection();}
    //public void setRobotRadius(double radius) {robot.setRadius(radius);}
	public void setRobotsRadius(double radius) {
		for(Robot r: robots) {
			r.setRadius(radius);
		}
	}
    public void refreshGraph() {
    	robot.getMap().refreshGraph();
		for(Robot r: robots)
    		r.setMapChangedSignal(true);
    }
	public int getRectWeight() {return rectWeight;}
	public void setRectWeight(int rectWeight) {this.rectWeight=rectWeight;}
	public void changeRectangle(int x, int y) {
		if(robot.getMap().getPassabilityMap()==null)
			return;
		Point point=robot.getMap().getCellCenterPoint(x, y);
		if(point==null)
			return;
		int scale=robot.getMap().getScale();
		for(int yi=point.y-scale/2, yBorder=point.y+scale/2; yi<yBorder; ++yi) {
			for(int xi=point.x-scale/2, xBorder=point.x+scale/2; xi<xBorder; ++xi) {
				robot.getMap().setPassabilityPoint(xi, yi, rectWeight);
			}
		}
		robot.getMap().calculatePassapilityForCell(point.x, point.y, robot.getRadius());

		for(Robot r: robots) {
			r.setMapChangedSignal(true);
		}
		repaint();
	}
	public void calculatePassability(double robotRadius) {
		long time=System.currentTimeMillis();
		int nodesCount=robot.getMap().calculatePassability(robotRadius);

		removeRobots();
		removeFinishes();
        refreshScreen();
        repaint();

		JOptionPane.showMessageDialog(this, "Map generation completed in " + (System.currentTimeMillis() - time) + " ms\nThere are " + nodesCount + " nodes");
	}
	public void removeMap() {
		setMapImages(null);
	}
	@Override
	public void keyPressed(KeyEvent arg0) {
		if(robotsRunning)
			return;
		Node finish=robot.getFinish();
		switch(arg0.getKeyCode()) {
		case KeyEvent.VK_UP:
			if(stage==6) {
				if(robot.getX()!=-1000 && robot.getY()!=-1000)
					placeRobot((int)robot.getX(), (int)robot.getY()-1);
			}
			else if(stage==7)
				if(finish.getX()!=-1000 && finish.getY()!=-1000)
					setFinish((int)finish.getX(), (int)finish.getY()-1);
			break;
		case KeyEvent.VK_DOWN:
			if(stage==6) {
				if(robot.getX()!=-1000 && robot.getY()!=-1000)
					placeRobot((int)robot.getX(), (int)robot.getY()+1);
			}
			else if(stage==7)
				if(finish.getX()!=-1000 && finish.getY()!=-1000)
					setFinish((int)finish.getX(), (int)finish.getY()+1);
			break;
		case KeyEvent.VK_LEFT:
			if(stage==6) {
				if(robot.getX()!=-1000 && robot.getY()!=-1000)
					placeRobot((int)robot.getX()-1, (int)robot.getY());
			}
			else if(stage==7)
				if(finish.getX()!=-1000 && finish.getY()!=-1000)
					setFinish((int)finish.getX()-1, (int)finish.getY());
			break;
		case KeyEvent.VK_RIGHT:
			if(stage==6) {
				if(robot.getX()!=-1000 && robot.getY()!=-1000)
					placeRobot((int)robot.getX()+1, (int)robot.getY());
			}
			else if(stage==7)
				if(finish.getX()!=-1000 && finish.getY()!=-1000)
					setFinish((int)finish.getX()+1, (int)finish.getY());
			break;
		}
		
	}
	@Override
	public void keyReleased(KeyEvent arg0) {
		// TODO Auto-generated method stub
		
	}
	@Override
	public void keyTyped(KeyEvent arg0) {
		// TODO Auto-generated method stub
		
	}

	public void addRobot(Robot r) {
		robots.add(r);
//		for(Robot r1:robots) {
//			r.addRobot(r1);
//			r1.addRobot(r);
//		}
	}

	// returns index of selected robot
	public int removeRobot() {
		if(robots.size()<2)
			return 1;
		int i=robots.indexOf(robot)-1;
		if(i<0)
			i=0;
		robots.remove(robot);
		robot=robots.get(i);
		repaint();
		return i;
	}

	public void runRobots() {
		robotsRunning=true;
		class MyThread extends Thread {
			private Robot robot;

			public MyThread(Robot robot) {
				this.robot=robot;
			}
			public void run() {
				robot.move(/*new Node(robot.getX(), robot.getY(), robot.getAzimuth()),*/ /*new Node(finish.getX(), finish.getY(), finish.getDirection())*/);
			}
		}
		//Hypervisor hypervisor=new Hypervisor(robots);
		try {
			int n = robots.size();

			MyThread t[] = new MyThread[n];
			//long time=System.currentTimeMillis();
//			t[0]=new MyThread(robot);
//			t[0].start();
			for (int i = 0; i < n; i++) {
				t[i] = new MyThread(robots.get(i));
				t[i].start();
			}
			if(logging)
				Hypervisor.startLog();
			for (int i = 0; i < n; i++) t[i].join();
			Hypervisor.stopLog();

		} catch (Exception ex) { };

		robotsRunning=false;
		JOptionPane.showMessageDialog(this, "All robots have stopped");
	}
	public boolean selectRobot(int i) {
		if(i>=robots.size() || i<0)
			return false;
		robot=robots.get(i);
		//finish=robot.getFinish();
		repaint();
		return true;
	}
	public int getRobotCount() {return robots.size();}

	public void setLogging(boolean selected) {
		logging=selected;
	}
}
