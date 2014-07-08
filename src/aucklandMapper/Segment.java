package aucklandMapper;
/* Code for COMP261 Assignment
 * Name:
 * Usercode:
 * ID:
 */

import java.util.*;
import java.awt.Graphics;
import java.awt.Point;

/** Segment   */

public class Segment{

    private Road road;      // the road this segment is part of
    private double length;   // length of segment
    private Node startNode; // the intersection it starts at
    private Node endNode;   // the intersection it ends at
    private List<Location> coords = new ArrayList<Location>();  // coords for drawing

    /** Construct a new Segment object */
    public Segment(Road r, double l, Node start, Node end){
	road = r;
	length = l;
	startNode = start;
	endNode = end;
    }

    /** Construct a new Segment object from a line in the data file */
    public Segment(String line, Map<Integer, Road> roads, Map<Integer, Node> nodes){
	String[] values = line.split("\t");
	road = roads.get(Integer.parseInt(values[0]));
	length = Double.parseDouble(values[1]);
	startNode = nodes.get(Integer.parseInt(values[2]));
	endNode = nodes.get(Integer.parseInt(values[3]));
	
	for (int i = 4; i<values.length; i+=2){
	    double lat = Double.parseDouble(values[i]);
	    double lon = Double.parseDouble(values[i+1]);
	    coords.add(Location.newFromLatLon(lat, lon));
	}
    }

    public Road getRoad(){
	return road;
    }
    public double getLength(){
	return length;
    }
    public Node getStartNode(){
	return startNode;
    }
    public Node getEndNode(){
	return endNode;
    }

    public void addCoord(Location loc){
	coords.add(loc);
    }
    public List<Location> getCoords(){
	return coords;
    }

    public Segment reverse(){
	Segment ans =  new Segment(road, length, endNode, startNode);
	ans.coords = this.coords;
	return ans;
    }


    /** draw the roadsegment on the graphics.
      For each location, shift the origin to origin and scale by scale*/
    public void draw(Graphics g, Location origin, double scale){
	if (!coords.isEmpty()){
	    //System.out.printf("Drawing road %d between nodes %d and %d%n", roadID, startNodeID, endNodeID);
	    Point p1 = coords.get(0).getPoint(origin, scale);
	    for (int i=1; i<coords.size(); i++){
		Point p2 = coords.get(i).getPoint(origin, scale);
		//System.out.printf("(%d,%d) to (%d,%d)%n",p1.x, p1.y, p2.x, p2.y);
		g.drawLine(p1.x, p1.y, p2.x, p2.y);
		p1=p2;
	    }	    
	}
    }

    public String toString(){
	return String.format("%d: %4.2fkm from %d to %d", road.getID(), length, startNode.getID(),endNode.getID());
    }

    
    public static void main(String[] arguments){
	AucklandMapper.main(arguments);
    }	
}
