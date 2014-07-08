package aucklandMapper;

/* Code for COMP261 Assignment
 * Name:
 * Usercode:
 * ID:
 */

import java.awt.Graphics;
import java.awt.Point;
import java.util.HashSet;
import java.util.Set;

/** Node */

public class Node {

	private int id;
	private Location loc; // coordinates of the intersection
	private Set<Segment> outNeighbours = new HashSet<Segment>();
	private Set<Segment> inNeighbours = new HashSet<Segment>();
	private boolean visited;
	private Node pathFrom;
	private Segment edge;
	private double cost;
	private int depth;

	/** Construct a new Node object */
	public Node(int id, Location l) {
		this.id = id;
		loc = l;
	}

	/** set the edge used to connect this node to origin */
	public void setEdge(Segment edge) {
		this.edge = edge;
	}

	/** get the edge used to connect this node to origin */
	public Segment getEdge() {
		return edge;
	}

	/** get current depth */
	public int getDepth() {
		return depth;
	}

	/** set the current depth of this node */
	public void setDepth(int depth) {
		this.depth = depth;
	}

	/** give a cost from origin to this node */
	public void setCost(double cost) {
		this.cost = cost;
	}

	/** get a cost from origin to this node */
	public double getCost() {
		return cost;
	}

	/** Give a track to this node */
	public void setVisited(boolean v) {
		if (v == false) {
			pathFrom = null;
			cost = 0;
		}
		visited = v;
	}

	/** is the node visited already ? */
	public boolean isVisited() {
		return visited;
	}

	/** set an origin node to this node */
	public void setPathFrom(Node pathFrom) {
		this.pathFrom = pathFrom;
	}

	/** Construct a new Node object from a line in the data file */
	public Node(String line) {
		String[] values = line.split("\t");
		id = Integer.parseInt(values[0]);
		double lat = Double.parseDouble(values[1]);
		double lon = Double.parseDouble(values[2]);
		loc = Location.newFromLatLon(lat, lon);
		// System.out.printf("Created Node %6d %s%n", id, loc);
	}

	public int getID() {
		return id;
	}

	public Location getLoc() {
		return this.loc;
	}

	public void addInSegment(Segment seg) {
		inNeighbours.add(seg);
	}

	public void addOutSegment(Segment seg) {
		outNeighbours.add(seg);
	}

	public Set<Segment> getOutNeighbours() {
		return outNeighbours;
	}

	public Set<Segment> getInNeighbours() {
		return inNeighbours;
	}

	public boolean closeTo(Location place, double dist) {
		return loc.closeTo(place, dist);
	}

	public double distanceTo(Location place) {
		return loc.distanceTo(place);
	}

	public void draw(Graphics g, Location origin, double scale) {
		Point p = loc.getPoint(origin, scale);
		g.fillRect(p.x, p.y, 2, 2);
	}

	public String toString() {
		StringBuilder b = new StringBuilder(String.format(
				"Intersection %d: at %s; Roads:  ", id, loc));
		Set<String> roadNames = new HashSet<String>();
		for (Segment neigh : inNeighbours) {
			roadNames.add(neigh.getRoad().getName());
		}
		for (Segment neigh : outNeighbours) {
			roadNames.add(neigh.getRoad().getName());
		}
		for (String name : roadNames) {
			b.append(name).append(", ");
		}
		return b.toString();
	}

}
