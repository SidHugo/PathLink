package ru.mcst.RobotGroup.PathsFinding;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

class SearchAlgorithm {
	private List<NodeForSearch> openList;
    private List<NodeForSearch> closeList;
    private List<Node> pathInNodes;
	private List<Link> pathInLinks;

	private static class NodeForSearch {
		private Node node;
		private double F=0, H=0, G=0;
		private NodeForSearch parent=null;

		public NodeForSearch(Node n, Node finish, double robotRadius, byte[][] passabilityArray) {
			node=n;
			calculateH(finish, robotRadius, passabilityArray);
		}
		public void calculateH(Node finish, double robotRadius, byte[][] passabilityArray) {
			int weight;
			if(passabilityArray==null)
				weight=0;
			else {
				if(0<=finish.getX() && finish.getX()<passabilityArray.length &&
						0<=finish.getY() && finish.getY()<passabilityArray[0].length) {
					weight=(127-passabilityArray[(int)finish.getX()][(int)finish.getY()]+127-passabilityArray[(int)node.getX()][(int)node.getY()])/2;
				} else {
					weight=254;
				}
			}
			Segment[][] s=new Segment[4][3];
			double minLength=-1, length=-1;
			int minIndex=-1;
			for(int i=0; i<4; ++i)
			{
				boolean isClockwise1=(i<2);
				boolean isClockwise2=(i==1 || i==3);
				if((s[i]=computeSegments(s[i], finish, isClockwise1, isClockwise2, robotRadius, -1, /*passabilityMap,*/ passabilityArray))!=null) {
					length=s[i][0].getLength()+s[i][1].getLength()+s[i][2].getLength();
				} else {
					length=-1;
				}
				if(length!=-1 && (minLength==-1 || minLength>length)) {
					minIndex=i;
					minLength=length;
				}
			}
			if(minIndex==-1)
				H=Double.MAX_VALUE;
			if(s[minIndex]!=null)
			{
				H=(s[minIndex][0].getLength()+s[minIndex][1].getLength()+s[minIndex][2].getLength())*(weight+1);
				// turning penalty
				H+=(s[minIndex][0].getRadiansTotal()+s[minIndex][2].getRadiansTotal())*(weight+1);
			}
			F=G+H;
		}
		private Segment[] computeSegments(Segment[] segments, Node n, boolean isClockwiseA, boolean isClockwiseB, double radius, int robotSize, byte[][] passabilityArray) {
			Segment seg0 = new Segment(), seg1 = new Segment(), seg2 = new Segment();

			seg0.setIsStraightLine(false);
			seg0.setIsClockwise(isClockwiseA);
			seg0.setRadius(radius);
			seg2.setIsStraightLine(false);
			seg2.setIsClockwise(isClockwiseB);
			seg2.setRadius(radius);

			seg0.setStartAngle(isClockwiseA? Segment.CapRadian(node.getDirection()+Segment.halfPI) : Segment.CapRadian(node.getDirection()-Segment.halfPI));

			seg0.setOriginX(node.getX()-radius*Math.cos(seg0.getStartAngle()));
			// changed sign, because of screen coordinates
			seg0.setOriginY(node.getY()+radius*Math.sin(seg0.getStartAngle()));

			double radStopB=(isClockwiseB? Segment.CapRadian(n.getDirection()+Segment.halfPI) : Segment.CapRadian(n.getDirection()-Segment.halfPI));

			seg2.setOriginX((double)n.getX()-radius*Math.cos(radStopB));
			// changed sign, because of screen coordinates
			seg2.setOriginY((double)n.getY()+radius*Math.sin(radStopB));

			// may be for some optimization
			double originX0=seg0.getOriginX();
			double originY0=seg0.getOriginY();
			double originX2=seg2.getOriginX();
			double originY2=seg2.getOriginY();
			if(!(originX0==originX2 && originY0==originY2 && isClockwiseA==isClockwiseB)) {
				double radStopA=Node.findTouchPoints(originX0, originY0, originX2, originY2,
						isClockwiseA, isClockwiseB, radius, seg1);
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

		public double getF() {
			return F;
		}

		public double getH() {
			return H;
		}

		public double getG() {
			return G;
		}

		public void setParent(NodeForSearch parent, double distanceToParent, int linkWeight, double linkRadiansTotal) {
			this.parent=parent;
			G=calculateG(distanceToParent, linkWeight, linkRadiansTotal, parent.getG());
			F=G+H;
		}
		public List<Link> getLinks() {return node.getLinks();}
		public Node getNode() {return node;}
		public static double calculateG(double distanceToParent, int linkWeight, double radiansTotal, double parentG) {
			return distanceToParent*(double)(linkWeight+1)+parentG+radiansTotal*(double)(linkWeight+1);
		}
		public NodeForSearch getParent() {return parent;}
		@Override
		public boolean equals(Object other){
			if (other == null) return false;
			if (other == this) return true;
			if(!(other instanceof NodeForSearch)) return false;
			return (node==((NodeForSearch)other).getNode());
		}
	}

	private List<NodeForSearch> nodes=new ArrayList<NodeForSearch>();
	
	public SearchAlgorithm() {
		openList=new ArrayList<NodeForSearch>();
		closeList=new ArrayList<NodeForSearch>();
		pathInNodes=new ArrayList<Node>();
		pathInLinks=new ArrayList<Link>();
	}
	public boolean searchAStar(Node n_start, Node n_finish, int robotSize, double robotRadius, byte[][] passabilityArray,
							   Robot searchingRobot, Robot blockingRobot, boolean findExact)
	{
		clean();
		
		if(n_start!=null && n_finish!=null) {
			openList.add(new NodeForSearch(n_start, n_finish, robotRadius, passabilityArray));
			NodeForSearch finishForSearch=iterationsAStar(n_finish, robotSize, passabilityArray, searchingRobot, blockingRobot);
			if(finishForSearch!=null)
			{
				fillPath(finishForSearch);
				fillPathInLinks();
				return true;
			}
			else if(!findExact)
			{
				List<Node> neighbours=searchingRobot.getMap().getNodesByChild24(n_finish, 0);
				for(Node n: neighbours) {
					if(n.getX()!=n_start.getX() && n.getY()!=n_start.getY() && !Hypervisor.isPointOccupiedAsFinish(n.getX(), n.getY(), searchingRobot)) {
						clean();
						openList.add(new NodeForSearch(n_start, n, robotRadius, passabilityArray));
						finishForSearch = iterationsAStar(n, robotSize, passabilityArray, searchingRobot, blockingRobot);
						if(finishForSearch!=null)
						{
							fillPath(finishForSearch);
							fillPathInLinks();
							return true;
						}
					}
				}
			}
		}
		return false;
	}
	private NodeForSearch iterationsAStar(Node n_finish, int robotSize, byte[][] passabilityArray,
										  Robot searchingRobot, Robot blockingRobot) {
		if(blockingRobot!=null) {
			if(Hypervisor.checkRobotInCoordinates((int)n_finish.getX(), (int)n_finish.getY(), robotSize, searchingRobot)!=null)
				return null;
		}
		while(!openList.isEmpty())
		{
			double minF=openList.get(0).getF();
			int index=0;
			
			for(int i=0; i<openList.size();++i)
			{
				if(openList.get(i).getF()<minF)
				{
					minF=openList.get(i).getF();
					index=i;
				}
					
			}
						
			NodeForSearch currentNode=openList.get(index);
			openList.remove(index);
			closeList.add(currentNode);			
			ConcurrentLinkedQueue<Link> links;
			synchronized (currentNode.getLinks()) {
				links = new ConcurrentLinkedQueue<Link>(currentNode.getLinks());
			}
			for(Iterator<Link> it=links.iterator(); it.hasNext();)
			{
				Link l=it.next();
				NodeForSearch newNode = new NodeForSearch(l.getChild(), n_finish, robotSize, passabilityArray);
				if(!closeList.contains(newNode))
				{
					int indexOfNewNode;
					if((indexOfNewNode=openList.indexOf(newNode))==-1)
					{
						if(Link.isSegmentsBlocked(l.getSegments(), robotSize, passabilityArray, l)) {
							currentNode.getNode().removeLink(l);
							it.remove();
							continue;
						}
						if(blockingRobot!=null && searchingRobot!=null) {
							if(Hypervisor.checkRobotsOnWay(searchingRobot, l, 0)!=null) {
								it.remove();
								continue;
							}
						}

						openList.add(newNode);
						newNode.setParent(currentNode, l.getLength(), l.getWeight(), l.getRadiansTotal());
						if (newNode.getNode()==n_finish)
							return newNode;
					}
					else
					{
						if(Link.isSegmentsBlocked(l.getSegments(), robotSize, passabilityArray, l)) {
							currentNode.getNode().removeLink(l);
							it.remove();
							continue;
						}
						if(blockingRobot!=null && searchingRobot!=null) {
							if(Hypervisor.checkRobotsOnWay(searchingRobot, l, 0)!=null) {
								it.remove();
								continue;
							}
						}

						if(NodeForSearch.calculateG(l.getLength(), l.getWeight(), l.getRadiansTotal(), currentNode.getG())<newNode.getG())
						{
							openList.get(indexOfNewNode).setParent(currentNode, l.getLength(), l.getWeight(), l.getRadiansTotal());
						}
					}
				}
			}
		}
		return null;
	}
	private void clean() {
		openList.clear();
		closeList.clear();
		pathInNodes.clear();
		pathInLinks.clear();
	}
    public List<Node> getPath() {return pathInNodes;}
    public boolean hasPath() {return !pathInNodes.isEmpty();}
    private void fillPath(NodeForSearch finish) {
    	pathInNodes.add(0, finish.getNode());
    	NodeForSearch n=finish.getParent();
    	while(n!= null){
    		pathInNodes.add(0, n.getNode());
    		n=n.getParent();
    	}    		
    }
	private void fillPathInLinks() {
		if(!pathInNodes.isEmpty()) {
			Node fromNode=pathInNodes.get(0);
			for(int i=1; i<pathInNodes.size(); ++i) {
				pathInLinks.add(fromNode.getLinkByChild(pathInNodes.get(i)));
				fromNode=pathInNodes.get(i);
			}
		}
	}

	public List<Link> getPathInLinks() {
		return pathInLinks;
	}
	public List<Link> getNextLinks(Link currentLink, int amount) {
		int index=pathInLinks.indexOf(currentLink);
		if(index!=-1) {
			List<Link> result=new ArrayList<Link>();
			for(int i=index; i<pathInLinks.size() && amount>0; ++i, --amount) {
				result.add(pathInLinks.get(i));
			}
			return result;
		}
		return null;
	}
}
