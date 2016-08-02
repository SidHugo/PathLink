package ru.mcst.RobotGroup.PathsFinding;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Semaphore;


class Robot implements Cloneable {
	public static final int ROBOT_NOWHERE_X=-1000;
	public static final int ROBOT_NOWHERE_Y=-1000;
	private final int speedDivider = 100;
	private final int sleepTime=1000/speedDivider;


	private MapInfo map;
	private double x;
    private double y;
	private double speed;

	private long updateTime=0;

	private double cachedX;
	private double cachedY;
	private  double cachedSpeed;
	private Semaphore semCachedSpeedCoordinates=new Semaphore(1);
	private int maxSpeed;
	private int minSpeed;
	private double acceleration;
	// 
	private double deceleration=1;
	private boolean maxSpeedSignal;
	private SearchAlgorithm searchPathAlgorithm;
	private boolean stopSignal;
	private boolean mapChangedSignal;
	private int sensorsRange=1;
	
	//RADIANS
	private double azimuth=0;
	
	private double radius=30;
	
	private boolean internalStopSignal=false;
	
	private List<Node> nodesToDelete=new ArrayList<Node>();

	private Node standing=null;
	private Node finish=new Node(ROBOT_NOWHERE_X, ROBOT_NOWHERE_Y, 0);

	private Robot robotBlocking=null;

	public Robot() {
		map=new MapInfo();
        x=ROBOT_NOWHERE_X;
        y=ROBOT_NOWHERE_Y;
        speed=0;
		cachedX=x;
		cachedY=y;
		cachedSpeed=speed;
        maxSpeed=100;
        minSpeed=40;
        acceleration=1;
        maxSpeedSignal=true;
        searchPathAlgorithm=new SearchAlgorithm();
        stopSignal=false;
        mapChangedSignal=false;
	}
	public Robot(MapInfo map) {
        this.map=map;
        x=ROBOT_NOWHERE_X;
        y=ROBOT_NOWHERE_Y;
        speed=0;
		cachedX=x;
		cachedY=y;
		cachedSpeed=speed;
        maxSpeed=100;
        minSpeed=40;
        acceleration=1;
        maxSpeedSignal=true;
        searchPathAlgorithm=new SearchAlgorithm();
        stopSignal=false;
        mapChangedSignal=false;
	}
	public Robot clone() throws CloneNotSupportedException {
		Robot obj=(Robot)super.clone();
		obj.searchPathAlgorithm=new SearchAlgorithm();
		obj.nodesToDelete=new ArrayList<Node>();
		obj.finish=new Node(ROBOT_NOWHERE_X, ROBOT_NOWHERE_Y, 0);
		obj.standing=null;
		obj.robotBlocking=null;
		return obj;
	}
	public long move() {
		Node start = new Node(x, y, azimuth);
    	if(start.getX()==ROBOT_NOWHERE_X || finish.getX()==ROBOT_NOWHERE_X) {
    		stopSignal=true;
    		return 0;
    	}
    	if(start.getX()==finish.getX() && start.getY()==finish.getY() &&
    	   start.getDirection()==finish.getDirection()) {
            stopSignal=true;
            return 0;
        }
    	
    	start.setIsRobotMade(true);
    	map.addNode(start);
		map.addLinksAroundCell24(start, radius, true);
		standing=start;
		
		Node toDelete=start;
		Node finishToDelete=finish;
		
		finish.setIsRobotMade(true);
		map.addNode(finish);
		map.addLinksAroundCell24(finish, radius, true);

		if(Math.abs(start.getX()-finish.getX())<=map.getScale() &&
		   Math.abs(start.getY()-finish.getY())<=map.getScale()) {
			start.addNeighbor(finish, radius, map.getScale(), map.getPassabilityArray());
		}
		
		stopSignal=false;
		mapChangedSignal=true;
		
    	internalStopSignal=false;
        
        long time=0, tempTime=0;
		int triesCounter=3;
		boolean findExact=false;
        while(true){
			if (mapChangedSignal) {
				mapChangedSignal=false;
            	if(time==0)
            		tempTime=System.currentTimeMillis();
				if(robotBlocking==null) {
					searchPathAlgorithm.searchAStar(standing, finish, map.getScale(), radius, map.getPassabilityArray(), this, null, findExact);
					if(searchPathAlgorithm.hasPath()) {
						triesCounter=3;
					}
				} else {
					if(Math.abs(x-finish.getX())<=2*map.getScale() && Math.abs(y-finish.getY())<=2*map.getScale())
						searchPathAlgorithm.searchAStar(standing, finish, map.getScale(), radius, map.getPassabilityArray(), this, robotBlocking, true);
					else
						searchPathAlgorithm.searchAStar(standing, finish, map.getScale(), radius, map.getPassabilityArray(), this, robotBlocking, false);
					if(searchPathAlgorithm.hasPath()) {
						triesCounter=3;
						robotBlocking=null;
					}
				}
            	if(time==0)
            		time=System.currentTimeMillis()-tempTime;
               internalStopSignal=false;
            }

			for (Link l: searchPathAlgorithm.getPathInLinks()) {
            	boolean speedZeroFlag=false;
            	if(mapChangedSignal) {
                    break;
                }
                else {
					speedZeroFlag=makeSteps(l, false);
                }
            	if(speedZeroFlag==true)
            		break;
            }
			if(standing==finish)
            {
            	speed=0;
            	map.removeNode(start);
                map.removeNode(finish);
                map.removeNode(toDelete);
				robotBlocking=null;
				cacheSpeedCoordinates();
            	return time;
            }
            if(!mapChangedSignal)	{
            	makeNodeUnderRobot();
            	cleanup(toDelete, finishToDelete);
				toDelete=standing;
				finishToDelete=finish;
				// if we did not manually stopped robot
                if(!stopSignal) {
					makeSmoothStop();
					if (robotBlocking != null || standing != finish) {
						mapChangedSignal = true; // to research path
						stopSignal = false; // to move
						findExact = true;
						try {
							Thread.sleep(500);
						} catch (InterruptedException e) {
							e.printStackTrace();
						}
						if (triesCounter > 0) {
							--triesCounter;
							makeNodeUnderRobot();
							toDelete = standing;
							continue;
						}
					}
				} else {	// i need this tautology because makeSmoothStop sets stopSignal and i need to check it before
					makeSmoothStop();
				}
				map.removeNode(start);
                map.removeNode(finish);
                map.removeNode(toDelete);
				cacheSpeedCoordinates();
            	return time;
            }
                        
            makeNodeUnderRobot();
            cleanup(toDelete, finishToDelete);
			toDelete=standing;
			finishToDelete=finish;
        }        
    }
    private void cleanup(Node toDelete, Node finishToDelete) {
		if(toDelete!=null)
			if(toDelete.getIsRobotMade())
    			map.removeNode(toDelete);
		if(nodesToDelete.size()!=0)
			synchronized (map.getNodes()) {
				for(Node n: nodesToDelete) {
					map.removeNode(n);
				}
			}
		nodesToDelete.clear();
    }
    private void makeSmoothStop() {
    	mapChangedSignal=false; // it was set, when a made "makeNodeUnderRobot"
    	stopSignal=true;
		Node toDelete=standing;
    	while(speed>0) {
			List<Link> links=standing.getLinks();
    		if(links!=null && links.size()==0)
    			break;
			Link minLink=null;
    		synchronized (links) {
    			for(Link l: links) {
					if(Hypervisor.checkRobotsOnWay(this, l, 0)==null) {
						if(minLink==null) {
							minLink=l;
							continue;
						}
						if (minLink.getChild().getLinks().size() == 0 && l.getChild().getLinks().size() != 0) {
							minLink = l;
						}
					}
	    		}
    		}
			if(minLink==null)
				break;
    		if(makeSteps(minLink, true))
    			break;
    		if(mapChangedSignal) {
				makeNodeUnderRobot();
				mapChangedSignal=false;
    		}
            cleanup(toDelete, null);
			toDelete=standing;
    	}
    	cleanup(toDelete, null);
    	speed=0;
    }
    // returns true when speed was changed and now its equals zero
    private boolean makeSteps(Link link, boolean isStopping) {
    	if(link==null)
    		return false;
    	double distance=0, globalDistance=0;
    	int i=0;
    	int step=map.getScale()/5;
    	int sensorCounter=0;
		long startedTime=System.currentTimeMillis();
		updateTime=System.currentTimeMillis();
    	while(globalDistance<link.getLength() && !mapChangedSignal){
    		if(sensorCounter==0) {
    			checkSensors();
				if(!isStopping) {
					if ((robotBlocking = Hypervisor.checkRobotsOnWay(this, searchPathAlgorithm.getNextLinks(link, 2), globalDistance)) != null) {
						mapChangedSignal = true;
						return false;
					}
				} else {
					if (Hypervisor.checkRobotsOnWay(this, link, globalDistance) != null) {
						mapChangedSignal = true;
						return true;
					}
				}
    			//sensorCounter=step;
				sensorCounter=1;
    		} else {
    			sensorCounter--;
    		}
    		if(mapChangedSignal)
    			return false;
    		changeSpeed();
    		if(internalStopSignal && speed==0) {
    			distance+=(double)1/ speedDivider;
    			globalDistance+=(double)1/ speedDivider;
    		} else {
    			distance+=speed/ speedDivider;
    			globalDistance+=speed/ speedDivider;
    		}
    		if(stopSignal && speed==0) {
				return true;
			}
    		Segment segment=link.getSegments()[i];
			if(distance<segment.getLength()) {
				if(!segment.getIsStraightLine()) {
					double theta;
					if(!segment.getIsClockwise()){
						theta=Segment.CapRadian(segment.getStartAngle()+distance/segment.getRadius());
						azimuth=Segment.CapRadian(theta+Math.PI/2);    						
					} else {
						theta=Segment.CapRadian(segment.getStartAngle()-distance/segment.getRadius());
						azimuth=Segment.CapRadian(theta-Math.PI/2);
					}
					x=segment.getOriginX()+segment.getRadius()*Math.cos(theta);
					y=segment.getOriginY()-segment.getRadius()*Math.sin(theta);
				} else {
					x=segment.getOriginX()+distance*Math.cos(segment.getStartAngle());
					y=segment.getOriginY()-distance*Math.sin(segment.getStartAngle());
					azimuth=segment.getStartAngle();
				}
				cacheSpeedCoordinates();
				try {    		            
					Thread.sleep((sleepTime-(System.currentTimeMillis()-startedTime ))< 0 ? 0 : (sleepTime-(System.currentTimeMillis()-startedTime)));
					startedTime=System.currentTimeMillis();
					updateTime=System.currentTimeMillis();
				} catch(InterruptedException ex) {
				    Thread.currentThread().interrupt();
				}
			} else {
				distance-=segment.getLength();
				++i;
				cacheSpeedCoordinates();
				if(i>=link.getSegments().length)
	    			break;
			}
    		checkForStop(globalDistance);
			if(speed==0 && !internalStopSignal)
				return true;
    	}
    	if(!mapChangedSignal){
			x=link.getChild().getX();
			y=link.getChild().getY();
			standing=link.getChild();
            azimuth=link.getChild().getDirection();
		}
    	return false;
    }

    private void checkSensors() {
    	if(map.getRealityMap()==null) {
    		checkForGrayBorderForward();
    		return;
    	}
    	double sinShift=Math.sin(azimuth+Segment.halfPI);
    	double cosShift=Math.cos(azimuth+Segment.halfPI);
    	double sinAzimuth=Math.sin(azimuth);
    	double cosAzimuth=Math.cos(azimuth);
    	double xi,yi;
    	List<Point> cells=new ArrayList<Point>();
    	for(int i=(-1)*map.getScale()/2; i<=map.getScale()/2; ++i) {
    		xi=x+i*cosShift - map.getScale()/2*cosAzimuth;
    		yi=y-i*sinShift + map.getScale()/2*sinAzimuth;
    		
    		for(int distance=0; distance<sensorsRange; ++distance) {
    			xi+=cosAzimuth;
    			yi-=sinAzimuth;
    			
    			if(xi<0 || xi>map.getRealityMap().getWidth() || yi<0 || yi>map.getRealityMap().getHeight()) {
    				checkForGrayBorderForward();
    	    		return;
    			}
    			Point cellCenter=map.getCellCenterPoint((int)xi,(int)yi);
    			if(cellCenter!=null && !cells.contains(cellCenter))
    				cells.add(cellCenter);
    			map.setPassabilityPoint((int)xi, (int)yi, map.getRealityWeight((int)xi, (int)yi));
    		}
    	}
    	boolean turnOffMaxSpeed=false;
    	for(Point cellCenter: cells) {
    		int oldWeight=map.getPassabilityWeight(cellCenter.x, cellCenter.y);
			int nodesToChangeSize=map.getNodes(cellCenter.x, cellCenter.y, 0).size();
    		map.calculatePassapilityForCell(cellCenter.x, cellCenter.y, radius);
    		int newWeight=map.getPassabilityWeight(cellCenter.x, cellCenter.y);
			int nodesChangedSize=map.getNodes(cellCenter.x, cellCenter.y, 0).size();
    		if((newWeight==255 && oldWeight!=255)
					|| (newWeight!=255 && oldWeight==255)
					|| Math.abs(oldWeight-newWeight)>MapColors.DIFFERENCE_BORDER
					|| (nodesToChangeSize!=0 && nodesChangedSize==0)
					|| (nodesToChangeSize==0 && nodesChangedSize!=0)) {
				Hypervisor.sendMapChangedSignal(true);
				mapChangedSignal = true;
			}
    		if(!turnOffMaxSpeed && newWeight>=MapColors.GRAY_BORDER)
    			turnOffMaxSpeed=true;
    	}
    	maxSpeedSignal=!turnOffMaxSpeed;
    }
	private void checkForGrayBorderForward() {
		double sinAzimuth=Math.sin(azimuth);
    	double cosAzimuth=Math.cos(azimuth);
    	double xi=x,yi=y;
		for(int distance=0; distance<sensorsRange; ++distance) {
			xi+=cosAzimuth;
			yi-=sinAzimuth;
			
			if(map.getPassabilityWeight((int)xi, (int)yi)>=MapColors.GRAY_BORDER) {
				maxSpeedSignal=false;
				return;
			}
		}
		maxSpeedSignal=true;		
	}
	private Robot checkRobotsBySensors() {
		double sinAzimuth=Math.sin(azimuth);
		double cosAzimuth=Math.cos(azimuth);
		double xi=x,yi=y;
		Robot robot=null;
		for(int distance=0; distance<sensorsRange; ++distance) {
			xi+=cosAzimuth;
			yi-=sinAzimuth;

			if((robot=(Hypervisor.checkRobotInCoordinates((int)xi, (int)yi, map.getScale(), this)))!=null) {
				return robot;
			}
		}
		return null;
	}
    public void setMap(MapInfo map) {this.map=map; mapChangedSignal=true;}
    public void setX(double x) {this.x=x;}
    public void setY(double y) {this.y=y;}
    public void setStopSignal(boolean signal) {stopSignal=signal;}
    public void setMapChangedSignal (boolean signal) {mapChangedSignal=signal;}
    public boolean hasPath() {return searchPathAlgorithm.hasPath();}
    public boolean isMapChangedSignal() {return mapChangedSignal;}
    public boolean isStopSignal() {return stopSignal;}
    public SearchAlgorithm getSearchAlgorithm() {return searchPathAlgorithm;}
    public double getX() {return x;}
    public double getY() {return y;}
    public double getSpeed() {return speed;}
    public void setSpeed(double speed) {this.speed=speed;}
    public MapInfo getMap() {return map;}
    public void setSensorsRange(int sensorsRange){
		this.sensorsRange=sensorsRange;
	}
    public int getSensorsRange() {return sensorsRange;}
    public void setMinSpeed(int minSpeed) {this.minSpeed=minSpeed;}
    public void setMaxSpeed(int maxSpeed) {this.maxSpeed=maxSpeed;}
    private void changeSpeed() {
    	if(stopSignal || internalStopSignal) {
    		if(speed>0) {
    			speed-=deceleration/ speedDivider;
    			if(speed<0)
    				speed=0;
    		}
    	} else {
	    	if(maxSpeedSignal) {
				if(speed<maxSpeed) {
					speed+=acceleration/ speedDivider;
					if(speed>maxSpeed)
						speed=maxSpeed;
				} else {
					speed-=deceleration/ speedDivider;
					if(speed<maxSpeed)
						speed=maxSpeed;
				}
			} else {
				if(speed>minSpeed) {
					speed-=deceleration/ speedDivider;
					if(speed<minSpeed)
						speed=minSpeed;
				} else {
					speed+=acceleration/ speedDivider;
					if(speed>minSpeed)
						speed=minSpeed;
				}
			}
    	}
    }
    private boolean makeNodeUnderRobot() {
    	Node n = new Node(x, y, azimuth);
		if(Link.getPointWeight((int)x, (int)y, azimuth, map.getScale(), map.getPassabilityArray())==255) {
			standing=null;
			return false;
		}
		n.setIsRobotMade(true);
		map.addNode(n);
        mapChangedSignal = true;
        map.addLinksAroundCell24(n, radius, true);
		standing=n;
        return true;
    }
    private int getIndexOfStanding() {
    	for(int i=0; i<searchPathAlgorithm.getPath().size(); ++i){
			if(searchPathAlgorithm.getPath().get(i)==standing) {
    			return i;    			
    		}
    	}
    	return -1;
    }
    private double calculatePathLength(double additionalDistance) {
    	double passed=additionalDistance, summary=0;
    	int indexOfStanding = getIndexOfStanding();
    	if(indexOfStanding==-1)
    		return -1;
    	Node fromNode=searchPathAlgorithm.getPath().get(0);
    	boolean calculatePassed=true;
    	for(int i=1; i<searchPathAlgorithm.getPath().size(); ++i) {    		
    		Node temp=searchPathAlgorithm.getPath().get(i);
    		// can be null, when node was deleted while moving
    		if(fromNode!=null && temp!=null) {
	    		if(calculatePassed)
	    			calculatePassed=!(fromNode==standing);
	    		if(calculatePassed)
	    			passed+=fromNode.getLinkByChild(temp).getLength();
	    		Link link=fromNode.getLinkByChild(temp);
	    		// can be null, when node was deleted while moving
	    		if(link!=null)
	    			summary+=fromNode.getLinkByChild(temp).getLength();
	    		fromNode=searchPathAlgorithm.getPath().get(i);
    		}
    	}
    	return summary-passed;
    }
    private void checkForStop(double additionalDistance) {
    	if(!stopSignal && !internalStopSignal) { 
    		// path calculating not from start to current point, but from current point to finish
	    	if((speed*speed)/(2*deceleration)>=(int)calculatePathLength(additionalDistance))
	    		internalStopSignal=true;
    	}
    }
    public void setAcceleration(double acceleration) {this.acceleration=acceleration;}
    public void setDeceleration(double deceleration) {this.deceleration=deceleration;}
    public void setAzimuth(double azimuth) {this.azimuth=azimuth;}
    public double getAzimuth() {return azimuth;}
    public double getRadius() {return radius;}
    public void setRadius(double radius) {this.radius=radius;}
    public void addNodeToDelete(Node node) {nodesToDelete.add(node);}
	public void setStanding(Node standing) {
		this.standing = standing;
	}
	public Node getFinish() {return finish;}
	public void setFinish(Node n) {
		finish=n;
		mapChangedSignal=true;
	}
	public Node getRealDestination() {
		List<Link> links=searchPathAlgorithm.getPathInLinks();
		if(links.size()!=0)
			return links.get(links.size()-1).getChild();
		else
			return null;
	}
	// blocking operation!
	private void cacheSpeedCoordinates() {
		try {
			semCachedSpeedCoordinates.acquire();
			cachedX=x;
			cachedY=y;
			cachedSpeed=speed;
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		semCachedSpeedCoordinates.release();
	}
	public double getCachedSpeed() {
		double result=-1;
		try {
			semCachedSpeedCoordinates.acquire();
			result=cachedSpeed;
			semCachedSpeedCoordinates.release();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		return result;
	}

	private String status = "";
	public double[] getCachedCoordinates() {
		double[] result=new double[2];
		try {
			semCachedSpeedCoordinates.acquire();
			result[0]=x;
			result[1]=y;
			semCachedSpeedCoordinates.release();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		return result;
	}

	public long getUpdateTime() {
		return updateTime;
	}
}


/****************************************************************************************************
****************************************************************************************************/