package ru.mcst.RobotGroup.PathsFinding;

import java.awt.Point;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

class Node {
	private double x;
	private double y;
	private List<Link> links=new ArrayList<Link>();
	private boolean isRobotMade;
	// RADIANS
	private double direction=0;

	public Node() {
        x=-1000;
		y=-1000;
        isRobotMade=false;
	}
	public Node(double x, double y, double direction) {
        this.x=x;
        this.y=y;
        this.direction=direction;
        isRobotMade=false;
	}
	// возвращает true, когда узел n является соседом данного узла
    public boolean isNeighbor(Node n) {
        synchronized (links) {
			for(Link l:links) {
	    		if(l.getChild()==n)
	    			return true;
	    	}
    	}
        return false;
    }
    public boolean addNeighbor(Node n, double radius, int robotSize, byte[][] passabilityArray) {
		if(isNeighbor(n) || this==n)
			return false;
		Segment s[][] = new Segment[4][3], s2[], sMin[] = null;
		double minLength=-1, length=-1;
		for(int i=0; i<4; ++i)
		{
			if((s2=computeSegments(s[i], n, (i<2), (i==1 || i==3), radius, robotSize, passabilityArray))!=null) {
				if(Link.isSegmentsBlocked(s2, robotSize, passabilityArray, null)) {
					s2=null;					
				} else {
					length=s2[0].getLength()+s2[1].getLength()+s2[2].getLength()+
						   s2[0].getRadiansTotal()+s2[2].getRadiansTotal();
				}
			} else {
				length=-1;
			}
			if(length!=-1 && length!=0 && (minLength==-1 || minLength>length)) {
				sMin = s2;
				minLength=length;
			}
		}
		if(sMin==null) return false;
		synchronized (links) {
			links.add(new Link(this, n, sMin));
		}
		return true;
	}
    private Segment[] computeSegments(Segment[] segments, Node n, boolean isClockwiseA, boolean isClockwiseB, double radius, int robotSize, byte[][] passabilityArray) {
		Segment seg0 = new Segment(), seg1 = new Segment(), seg2 = new Segment();

		seg0.setIsStraightLine(false);
		seg0.setIsClockwise(isClockwiseA);
		seg0.setRadius(radius);
		seg2.setIsStraightLine(false);
		seg2.setIsClockwise(isClockwiseB);
		seg2.setRadius(radius);
		
		seg0.setStartAngle(isClockwiseA? Segment.CapRadian(direction+Segment.halfPI) : Segment.CapRadian(direction-Segment.halfPI));
		
		seg0.setOriginX(x-radius*Math.cos(seg0.getStartAngle()));
		// changed sign, because of screen coordinates
		seg0.setOriginY(y+radius*Math.sin(seg0.getStartAngle()));
		
		double radStopB=(isClockwiseB? Segment.CapRadian(n.direction+Segment.halfPI) : Segment.CapRadian(n.direction-Segment.halfPI));
		
		seg2.setOriginX((double)n.getX()-radius*Math.cos(radStopB));
		// changed sign, because of screen coordinates
		seg2.setOriginY((double)n.getY()+radius*Math.sin(radStopB));
		
		// may be for some optimization
		double originX0=seg0.getOriginX();
		double originY0=seg0.getOriginY();
		double originX2=seg2.getOriginX();
		double originY2=seg2.getOriginY();
		if(!(originX0==originX2 && originY0==originY2 && isClockwiseA==isClockwiseB)) { 
			double radStopA=findTouchPoints(originX0,originY0,originX2,originY2,
											isClockwiseA,isClockwiseB,radius,seg1);
		    if (seg1.getLength() < 0)
		        return null;
	
		    seg0.setRadiansTotal(isClockwiseA ? Segment.CapRadian(seg0.getStartAngle() - radStopA) 
		                                                          : Segment.CapRadian(radStopA - seg0.getStartAngle()));
		    seg0.setLength(seg0.getRadiansTotal() * radius);
		    seg2.setStartAngle(isClockwiseA  == isClockwiseB ? 
		                              radStopA : Segment.CapRadian(radStopA + Math.PI));
		    seg2.setRadiansTotal(isClockwiseB ? Segment.CapRadian(seg2.getStartAngle() - radStopB) 
		                                                         	: Segment.CapRadian(radStopB - seg2.getStartAngle()));
		    seg2.setLength(seg2.getRadiansTotal() * radius);
	
		    // Finish information on the straight line segment (length already set above)
		    seg1.setIsStraightLine(true);
		    seg1.setOriginX(originX0 + radius* Math.cos(radStopA));
		    // changed sign, because of screen coordinates
		    seg1.setOriginY(originY0 - radius* Math.sin(radStopA));
		    seg1.setStartAngle(isClockwiseA? Segment.CapRadian(radStopA - Segment.halfPI) : Segment.CapRadian(radStopA + Segment.halfPI)); 
		    seg1.setRadiansTotal(0);
		} else {
			seg0.setRadiansTotal(isClockwiseA ? Segment.CapRadian(seg0.getStartAngle() - radStopB) 
			                    								     : Segment.CapRadian(radStopB - seg0.getStartAngle()));
			seg0.setLength(seg0.getRadiansTotal() * radius);
			seg2.setStartAngle(isClockwiseA  == isClockwiseB ? 
									  radStopB : Segment.CapRadian(radStopB + Math.PI));
			seg2.setRadiansTotal(0);
			seg2.setLength(0);
			
			// Finish information on the straight line segment (length already set above)
			seg1.setIsStraightLine(true);
			seg1.setOriginX(originX0 + radius* Math.cos(radStopB));
			// changed sign, because of screen coordinates
			seg1.setOriginY(originY0 - radius* Math.sin(radStopB));
			seg1.setStartAngle(isClockwiseA? Segment.CapRadian(radStopB - Segment.halfPI) : Segment.CapRadian(radStopB + Segment.halfPI)); 
			seg1.setRadiansTotal(0);
			seg1.setLength(0);
		}
		segments[0] = seg0; segments[1] = seg1; segments[2] = seg2;
		return segments;
	}
	public static double findTouchPoints(double originXA, double originYA, double originXB, double originYB,
			   boolean isClockwiseA, boolean isClockwiseB, double radius, Segment line) {
		double dx=originXB-originXA;
		// 	changed sign, because of screen coordinates
		double dy=originYA-originYB;
		double angleOrigin, angleFinal;
		
		angleOrigin=Math.atan2(dy, dx);
		
		line.setLength(Math.sqrt(dx*dx+dy*dy));
		
		if(isClockwiseA==isClockwiseB)
		{
			if(isClockwiseA)
				return Segment.CapRadian(angleOrigin+Segment.halfPI);
			else
				return Segment.CapRadian(angleOrigin-Segment.halfPI);
		}
		else if(Math.abs(2*radius/line.getLength())<=1)
		{
			double angleRight = Math.acos(2*radius/line.getLength());
			angleFinal=Segment.CapRadian(isClockwiseA? angleOrigin+angleRight : angleOrigin-angleRight);
			double xNew=originXA+2*radius*Math.cos(angleFinal);
			// 	changed sign, because of screen coordinates
			double yNew=originYA-2*radius*Math.sin(angleFinal);
			dx=originXB-xNew;
			// 	changed sign, because of screen coordinates
			dy=yNew-originYB;
			line.setLength(Math.sqrt(dx*dx+dy*dy));
			
			return angleFinal;
		}
		else
		{
			line.setLength(-1);
			return 0;
		}
	}	
	public double getX() {return x;}
    public double getY() {return y;}
    public void setX(double x) {this.x=x;}
    public void setY(double y) {this.y=y;}
	public List<Link> getLinks() {return links;}
	public boolean removeNeighbor(Node n) {synchronized (links) {
			for(Iterator<Link> it=links.iterator(); it.hasNext();) {
				Link l=it.next();
				if(l.getChild()==n) {
					it.remove();
				}
			}
		}
		return true;
	}
	public void setIsRobotMade(boolean isRobotMade) {this.isRobotMade=isRobotMade;}
	public boolean getIsRobotMade() {return isRobotMade;}
	public double getDirection() {return direction;}
	public Link getLinkByChild(Node child) {
		synchronized (links) {
			for(Link l:links)
				if(l.getChild()==child)
					return l;
		}
		return null;
	}
	public Point getPoint() {return new Point((int)x, (int)y);}
	public void setDirection(double direction) {this.direction=direction;}
	public void removeLink(Link link) {
		synchronized (links) {
			links.remove(link);
		}
	}
}
