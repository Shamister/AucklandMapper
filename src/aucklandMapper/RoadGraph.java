package aucklandMapper;

/* Code for COMP261 Assignment
 * Name:
 * Usercode:
 * ID:

 */

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Point;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.Stack;

/** RoadMap: The list of the roads and the graph of the road network */

public class RoadGraph {

	private double averageSpeed; // the average speed we get from roads
	private double maxSpeed; // tha max speed we get from roads
	private final double TRAFFIC_COST = 600; // in seconds, we assume that it
												// takes approximately 10
												// minutes

	double westBoundary = Double.POSITIVE_INFINITY;
	double eastBoundary = Double.NEGATIVE_INFINITY;
	double southBoundary = Double.POSITIVE_INFINITY;
	double northBoundary = Double.NEGATIVE_INFINITY;

	// start and goal node
	Node start;
	Node goal;

	// visited segments
	Set<Segment> visitedPaths = new HashSet<Segment>();

	// visited segments mapped into the time consumed
	// this is for finding shortest time between 2 nodes only
	Map<Segment, Double> visitedPathsAndTimeCosts = new HashMap<Segment, Double>();

	// articulation points
	Set<Node> articulationPoints = new HashSet<Node>();

	// the map containing the restrictions between 2 segments
	Map<Segment, Set<Segment>> restrictions = new HashMap<Segment, Set<Segment>>();
	// total restrictions
	int totalRes = 0;

	// the map containing the graph of nodes (and road segments), indexed by the
	// nodeID
	Map<Integer, Node> nodes = new HashMap<Integer, Node>();

	// the map of roads, indexed by the roadID
	Map<Integer, Road> roads = new HashMap<Integer, Road>();

	// the map of roads, indexed by name
	Map<String, Set<Road>> roadsByName = new HashMap<String, Set<Road>>();

	Set<String> roadNames = new HashSet<String>();

	/** Construct a new RoadMap object */
	public RoadGraph() {
	}

	public String loadData(String dataDirectory) {
		// Read roads into roads array.
		// Read the nodes into the roadGraph array.
		// Read each road segment
		// put the segment into the neighbours of the startNode
		// If the road of the segment is not one way,
		// also construct the reversed segment and put it into
		// the neighbours of the endNode
		// Work out the boundaries of the region.
		String report = "";
		System.out.println("Loading roads...");
		loadRoads(dataDirectory);
		report += String.format(
				"Loaded %,d roads, with %,d distinct road names%n", roads
						.entrySet().size(), roadNames.size());
		System.out.println("Loading intersections...");
		loadNodes(dataDirectory);
		report += String.format("Loaded %,d intersections%n", nodes.entrySet()
				.size());
		System.out.println("Loading road segments...");
		loadSegments(dataDirectory);
		report += String.format("Loaded %,d road segments%n", numSegments());
		System.out.println("Loading restrictions...");
		loadRestrictions(dataDirectory);
		report += String.format("Loadded %,d restrictions%n", totalRes);
		return report;
	}

	public void loadRoads(String dataDirectory) {
		File roadFile = new File(dataDirectory + "roadID-roadInfo.tab");
		if (!roadFile.exists()) {
			System.out.println("roadID-roadInfo.tab not found");
			return;
		}
		BufferedReader data;
		try {
			data = new BufferedReader(new FileReader(roadFile));
			data.readLine(); // throw away header line.
			double totSpeed = 0;
			while (true) {
				String line = data.readLine();
				if (line == null) {
					break;
				}
				Road road = new Road(line);
				roads.put(road.getID(), road);
				// get the speed
				totSpeed += road.getSpeed();
				// compare the speed with max speed and get the faster one
				maxSpeed = (maxSpeed < road.getSpeed()) ? road.getSpeed()
						: maxSpeed;
				String fullName = road.getFullName();
				roadNames.add(fullName);
				Set<Road> rds = roadsByName.get(fullName);
				if (rds == null) {
					rds = new HashSet<Road>(4);
					roadsByName.put(fullName, rds);
				}
				rds.add(road);
			}
			// get average speed;
			averageSpeed = totSpeed / roads.values().size();
		} catch (IOException e) {
			System.out.println("Failed to open roadID-roadInfo.tab: " + e);
		}
	}

	public void loadNodes(String dataDirectory) {
		File nodeFile = new File(dataDirectory + "nodeID-lat-lon.tab");
		if (!nodeFile.exists()) {
			System.out.println("nodeID-lat-lon.tab not found");
			return;
		}
		BufferedReader data;
		try {
			data = new BufferedReader(new FileReader(nodeFile));
			while (true) {
				String line = data.readLine();
				if (line == null) {
					break;
				}
				Node node = new Node(line);
				nodes.put(node.getID(), node);
			}
		} catch (IOException e) {
			System.out.println("Failed to open roadID-roadInfo.tab: " + e);
		}
	}

	public void loadSegments(String dataDirectory) {
		File segFile = new File(dataDirectory
				+ "roadSeg-roadID-length-nodeID-nodeID-coords.tab");
		if (!segFile.exists()) {
			System.out
					.println("roadSeg-roadID-length-nodeID-nodeID-coords.tab not found");
			return;
		}
		BufferedReader data;
		try {
			data = new BufferedReader(new FileReader(segFile));
			data.readLine(); // get rid of headers
			while (true) {
				String line = data.readLine();
				if (line == null) {
					break;
				}
				Segment seg = new Segment(line, roads, nodes);
				// System.out.println(seg);
				Node node1 = seg.getStartNode();
				Node node2 = seg.getEndNode();
				node1.addOutSegment(seg);
				node2.addInSegment(seg);
				Road road = seg.getRoad();
				road.addSegment(seg);
				if (!road.isOneWay()) {
					Segment revSeg = seg.reverse();
					node2.addOutSegment(revSeg);
					node1.addInSegment(revSeg);
				}
			}
		} catch (IOException e) {
			System.out.println("Failed to open roadID-roadInfo.tab: " + e);
		}
	}

	/** Read restrictions.tab file */
	public void loadRestrictions(String dataDirectory) {
		File resFile = new File(dataDirectory + "restrictions.tab");
		if (!resFile.exists()) {
			System.out.println("restrictions.tab not found");
			return;
		}
		BufferedReader data;
		try {
			data = new BufferedReader(new FileReader(resFile));
			data.readLine(); // get rid of headers
			while (true) {
				String line = data.readLine();
				if (line == null) {
					break;
				}
				// here is the code
				// get segments and set of segments of restrictions
				// later on, we will compare whether it go through 1st segments
				// if so, then we check whether the second segment it want to go
				// is in the set of segments
				// (go through the restrictions)
				// if yes, then we should not allow it
				String[] tokens = line.split("\t");
				// get actual nodes and roads in collections
				Node node1 = nodes.get(Integer.parseInt(tokens[0]));
				Road road1 = roads.get(Integer.parseInt(tokens[1]));
				Node node2 = nodes.get(Integer.parseInt(tokens[2]));
				Road road2 = roads.get(Integer.parseInt(tokens[3]));
				Node node3 = nodes.get(Integer.parseInt(tokens[4]));

				// instantiate dummy segment
				Set<Segment> setOfSegs = new HashSet<Segment>();

				// check every single segment in road1
				// if it found the segment expected, then it stores to sTemp
				for (Segment s : node1.getOutNeighbours()) {
					if (s.getEndNode().equals(node2)
							&& s.getRoad().equals(road1)) {
						setOfSegs.add(s);
					}
				}

				// check every single segment in road2
				// if it found the segment expected, then we put the 1st and 2nd
				// segment into map
				// if the 1st segment is already exist, then we put 2nd segment
				// into the set of segments
				for (Segment sTemp : setOfSegs) {
					for (Segment s : node2.getOutNeighbours()) {
						if (s.getEndNode().equals(node3)
								&& s.getRoad().equals(road2)) {
							if (restrictions.containsKey(sTemp)) {
								restrictions.get(sTemp).add(s);
							} else {
								Set<Segment> setOfSegs2 = new HashSet<Segment>();
								setOfSegs2.add(s);
								restrictions.put(sTemp, setOfSegs2);
							}
							totalRes++;
						}
					}
				}
			}
		} catch (IOException e) {
			System.out.println("Failed to open restrictions.tab: " + e);
		}
	}

	public double[] getBoundaries() {
		double west = Double.POSITIVE_INFINITY;
		double east = Double.NEGATIVE_INFINITY;
		double south = Double.POSITIVE_INFINITY;
		double north = Double.NEGATIVE_INFINITY;

		for (Node node : nodes.values()) {
			Location loc = node.getLoc();
			if (loc.x < west) {
				west = loc.x;
			}
			if (loc.x > east) {
				east = loc.x;
			}
			if (loc.y < south) {
				south = loc.y;
			}
			if (loc.y > north) {
				north = loc.y;
			}
		}
		return new double[] { west, east, south, north };
	}

	public void checkNodes() {
		for (Node node : nodes.values()) {
			if (node.getOutNeighbours().isEmpty()
					&& node.getInNeighbours().isEmpty()) {
				System.out.println("Orphan: " + node);
			}
		}
	}

	public int numSegments() {
		int ans = 0;
		for (Node node : nodes.values()) {
			ans += node.getOutNeighbours().size();
		}
		return ans;
	}

	public void redraw(Graphics g, Location origin, double scale) {
		// System.out.printf("Drawing road graph. at (%.2f, %.2f) @ %.3f%n",
		// origX, origY, scale);
		g.setColor(Color.black);
		for (Node node : nodes.values()) {
			for (Segment seg : node.getOutNeighbours()) {
				seg.draw(g, origin, scale);
			}
		}

		for (Node node : nodes.values()) {
			if (articulationPoints.contains(node)) {
				g.setColor(Color.green);
			} else {
				g.setColor(Color.blue);
			}
			node.draw(g, origin, scale);
		}

		// visited paths
		g.setColor(Color.orange);
		for (Segment seg : visitedPaths) {
			seg.draw(g, origin, scale);
		}

		g.setColor(Color.pink);
		for (Segment seg : visitedPathsAndTimeCosts.keySet()) {
			seg.draw(g, origin, scale);
		}

		// start to goal paths
		// visited paths
		g.setColor(Color.cyan);
		boolean finish = false;
		Node temp = goal;
		if (start != null && goal != null) {
			while (!finish) {
				temp.getEdge().draw(g, origin, scale);
				if (temp.getEdge() != null) {
					temp = temp.getEdge().getStartNode();
					if (temp.equals(start)) {
						finish = true;
					}
				}
			}
		}

		// draw the articulation points here
	}

	private double mouseThreshold = 5; // how close does the mouse have to be?

	public Node findNode(Point point, Location origin, double scale) {
		Location mousePlace = Location.newFromPoint(point, origin, scale);
		/*
		 * System.out.printf("find at %d %d -> %.3f %.3f -> %d %d %n", point.x,
		 * point.y, x, y, (int)((x-origX)*scale),(int)((y-origY)*(-scale)) );
		 */
		Node closestNode = null;
		double mindist = Double.POSITIVE_INFINITY;
		for (Node node : nodes.values()) {
			double dist = node.distanceTo(mousePlace);
			if (dist < mindist) {
				mindist = dist;
				closestNode = node;
			}
		}
		return closestNode;
	}

	/** Returns information of roads it goes through by shortest distance */
	public List<String> roadsGoThroughByDist() {
		// instantiate the stack
		Stack<Segment> stackTemp = new Stack<Segment>();

		boolean finish = false;
		Node temp = goal;
		if (start != null && goal != null) {
			while (!finish) {
				stackTemp.add(temp.getEdge());
				if (temp.getEdge() != null) {
					temp = temp.getEdge().getStartNode();
					if (temp.equals(start)) {
						finish = true;
					}
				}
			}
		}

		// convert stack of segments to list of road name
		LinkedHashSet<String> listTemp = new LinkedHashSet<String>();
		for (Segment s : stackTemp) {
			if (!listTemp.contains(s.getRoad())) {
				listTemp.add(s.getRoad().getName());
			}
		}

		// convert stack of segments into a map which relate the road name
		// and its length passed from start to goal
		Map<String, Double> mapOfRoads = new HashMap<String, Double>();
		// if the road already exists, then just adding the length to the
		// previous one
		// otherwise create the new one and put it into map
		for (Segment s : stackTemp) {
			if (mapOfRoads.containsKey(s.getRoad().getName())) {
				mapOfRoads.put(s.getRoad().getName(),
						mapOfRoads.get(s.getRoad().getName()) + s.getLength());
			} else {
				mapOfRoads.put(s.getRoad().getName(), s.getLength());
			}
		}

		List<String> roadsInfo = new ArrayList<String>();

		double total = 0;
		for (String s : listTemp) {
			// split road names into several words based on the white space
			String[] roadNames = s.split(" ");
			String fullname = "";
			// rearrange the road name properly
			for (String str : roadNames) {
				switch (str) {
				case "rd":
					fullname += "Road ";
					break;
				case "st":
					fullname += "Street ";
					break;
				case "mt":
					fullname += "Mount ";
					break;
				case "cres":
					fullname += "Crescent ";
					break;
				default:
					fullname += str + " ";
				}
			}
			// capitalize the name
			fullname = fullname.substring(0, 1).toUpperCase()
					+ fullname.substring(1);
			roadsInfo.add(String.format("%s%.3f km", fullname,
					mapOfRoads.get(s)));

			// sum up the length
			total += mapOfRoads.get(s); // it is not rounded to get the decimal
										// point more precisely
		}

		// add details of total length
		roadsInfo.add("");
		roadsInfo.add(String.format("Total distance = %.3f km", total));

		return roadsInfo;
	}

	/** Returns information of roads it goes through by shortest distance */
	public List<String> roadsGoThroughByTime() {
		// instantiate the stack
		Stack<Segment> stackTemp = new Stack<Segment>();

		boolean finish = false;
		Node temp = goal;
		if (start != null && goal != null) {
			while (!finish) {
				stackTemp.add(temp.getEdge());
				if (temp.getEdge() != null) {
					temp = temp.getEdge().getStartNode();
					if (temp.equals(start)) {
						finish = true;
					}
				}
			}
		}

		// convert stack of segments to list of road name
		LinkedHashSet<String> listTemp = new LinkedHashSet<String>();
		for (Segment s : stackTemp) {
			if (!listTemp.contains(s.getRoad())) {
				listTemp.add(s.getRoad().getName());
			}
		}

		// convert stack of segments into a map which relate the road name
		// and its length passed from start to goal
		Map<String, Double> mapOfRoads = new HashMap<String, Double>();
		// if the road already exists, then just adding the length to the
		// previous one
		// otherwise create the new one and put it into map
		for (Segment s : stackTemp) {
			if (mapOfRoads.containsKey(s.getRoad().getName())) {
				mapOfRoads.put(s.getRoad().getName(),
						mapOfRoads.get(s.getRoad().getName())
								+ visitedPathsAndTimeCosts.get(s));
			} else {
				mapOfRoads.put(s.getRoad().getName(),
						visitedPathsAndTimeCosts.get(s));
			}
		}

		List<String> roadsInfo = new ArrayList<String>();

		double total = 0;
		for (String s : listTemp) {
			// split road names into several words based on the white space
			String[] roadNames = s.split(" ");
			String fullname = "";
			// rearrange the road name properly
			for (String str : roadNames) {
				switch (str) {
				case "rd":
					fullname += "Road ";
					break;
				case "st":
					fullname += "Street ";
					break;
				case "mt":
					fullname += "Mount ";
					break;
				case "cres":
					fullname += "Crescent ";
					break;
				default:
					fullname += str + " ";
				}
			}
			// capitalize the name
			fullname = fullname.substring(0, 1).toUpperCase()
					+ fullname.substring(1);
			roadsInfo.add(String.format("%s%.3f seconds", fullname,
					mapOfRoads.get(s)));

			// sum up the length
			total += mapOfRoads.get(s); // it is not rounded to get the decimal
										// point more precisely
		}

		// add details of total length
		roadsInfo.add("");
		roadsInfo.add(String.format(
				"Total estimated time = %.3f s (%.3f minutes)", total,
				total / 60));

		return roadsInfo;
	}

	/** find articulation points in the graph */
	public void findArticulationPoints() {
		// set all node depth to infinity
		for (Node node : nodes.values()) {
			node.setDepth(Integer.MAX_VALUE);
		}
		// go through all the nodes
		// and find the intersection points
		Iterator<Node> it = nodes.values().iterator();
		int numSubtrees = 0;
		while (it.hasNext()) {
			Node start = it.next();
			if (start.getDepth() == Integer.MAX_VALUE) {
				start.setDepth(0);
				for (Segment seg : start.getInNeighbours()) {
					Node neigh = seg.getStartNode();
					iterArtPoints(neigh, start);
					/* recArtPoints(neigh, 1, start); */
					numSubtrees++;
				}
				for (Segment seg : start.getOutNeighbours()) {
					Node neigh = seg.getEndNode();
					iterArtPoints(neigh, start);
					/* recArtPoints(neigh, 1, start); */
					numSubtrees++;
				}
			}
		}
		if (numSubtrees > 1) {
			articulationPoints.add(start);
		}
	}

	/** find articulation points with iterative version */
	public void iterArtPoints(Node firstNode, Node root) {
		// mini class
		class NodeParentDepth {
			public Node node;
			public NodeParentDepth parent;
			public int depth;
			public int reach;
			public Queue<Node> children;

			public NodeParentDepth(Node node, int depth, NodeParentDepth parent) {
				this.node = node;
				this.parent = parent;
				this.depth = depth;
			}
		}

		Stack<NodeParentDepth> stackNode = new Stack<NodeParentDepth>();

		stackNode.push(new NodeParentDepth(firstNode, 1, new NodeParentDepth(
				root, 0, null)));

		// do looping
		while (!stackNode.isEmpty()) {
			// get an element of stack
			NodeParentDepth elem = stackNode.peek();
			// get node of element
			Node node = elem.node;

			// if children has not been created
			// initialize and construct children
			if (elem.children == null) {
				// set elem depth to node depth
				node.setDepth(elem.depth);
				// set reach of elem to depth of elem (first time it will be
				// the same)
				elem.reach = elem.depth;
				// create new queue
				elem.children = new LinkedList<Node>();

				// add all neighbors into queue
				// since it does not care about direction
				// the neighbors from and to this node will be added
				for (Segment s : node.getOutNeighbours()) {
					Node neigh = s.getEndNode();
					if (!neigh.equals(elem.parent.node)) {
						elem.children.add(neigh);
					}
				}
				for (Segment s : node.getInNeighbours()) {
					Node neigh = s.getStartNode();
					if (!neigh.equals(elem.parent.node)) {
						elem.children.add(neigh);
					}
				}
			} else if (!elem.children.isEmpty()) {
				Node child = elem.children.poll();
				if (child.getDepth() < Integer.MAX_VALUE) {
					elem.reach = Math.min(elem.reach, child.getDepth());
				} else {
					stackNode.push(new NodeParentDepth(child,
							node.getDepth() + 1, elem));
				}
			} else {
				if (!node.equals(firstNode)) {
					if (elem.reach >= elem.parent.depth) {
						articulationPoints.add(elem.parent.node);
					}
					elem.parent.reach = Math.min(elem.parent.reach, elem.reach);
				}
				stackNode.pop();
			}

		}

	}

	/** find articulation points with recursive version */
	public int recArtPoints(Node node, int depth, Node fromNode) {
		// set the depth
		node.setDepth(depth);
		// set reach back to the same as its depth
		int reachBack = node.getDepth();

		for (Segment seg : node.getInNeighbours()) {
			Node neigh = seg.getStartNode();
			if (!neigh.equals(fromNode)) {
				if (neigh.getDepth() < Integer.MAX_VALUE) {
					reachBack = Math.min(neigh.getDepth(), reachBack);
				} else {
					int childReach = recArtPoints(neigh, depth + 1, node);
					reachBack = Math.min(childReach, reachBack);
					if (childReach >= depth) {
						articulationPoints.add(node);
					}
				}
			}
		}

		for (Segment seg : node.getOutNeighbours()) {
			Node neigh = seg.getEndNode();
			if (!neigh.equals(fromNode)) {
				if (neigh.getDepth() < Integer.MAX_VALUE) {
					reachBack = Math.min(neigh.getDepth(), reachBack);
				} else {
					int childReach = recArtPoints(neigh, depth + 1, node);
					reachBack = Math.min(childReach, reachBack);
					if (childReach >= depth) {
						articulationPoints.add(node);
					}
				}
			}
		}
		return reachBack;
	}

	public Set<Node> getArticulationPoints() {
		return articulationPoints;
	}

	/**
	 * Returns a set of full road names that match the query. If the query
	 * matches a full road name exactly, then it returns just that name
	 */
	public Set<String> lookupName(String query) {
		Set<String> ans = new HashSet<String>(10);
		if (query == null)
			return null;
		query = query.toLowerCase();
		for (String name : roadNames) {
			if (name.equals(query)) { // this is the right answer
				ans.clear();
				ans.add(name);
				return ans;
			}
			if (name.startsWith(query)) { // it is an option
				ans.add(name);
			}
		}
		return ans;
	}

	/** Get the Road objects associated with a (full) road name */
	public Set<Road> getRoadsByName(String fullname) {
		return roadsByName.get(fullname);
	}

	/**
	 * Return a list of all the segments belonging to the road with the given
	 * (full) name.
	 */
	public List<Segment> getRoadSegments(String fullname) {
		Set<Road> rds = roadsByName.get(fullname);
		if (rds == null) {
			return null;
		}
		System.out.println("Found " + rds.size() + " road objects: "
				+ rds.iterator().next());
		List<Segment> ans = new ArrayList<Segment>();
		for (Road road : rds) {
			ans.addAll(road.getSegments());
		}
		return ans;
	}

	/** mini class which stores information between 2 nodes */
	private class NodeToNode {
		Node node;
		Node from;
		double costToHere;
		double costToGoal;
		Segment edge;

		public NodeToNode(Node node, Node from, double costToHere,
				double costToGoal, Segment edge) {
			this.node = node;
			this.from = from;
			this.costToHere = costToHere;
			this.costToGoal = costToGoal;
			this.edge = edge;
		}
	}

	/** Comparator to compare total cost to goal between 2 nodes */
	public static Comparator<NodeToNode> toGoalComparator = new Comparator<NodeToNode>() {
		@Override
		public int compare(NodeToNode ntn1, NodeToNode ntn2) {
			return (ntn1.costToGoal < ntn2.costToGoal) ? -1 : 1;
		}
	};

	/** Find the shortest path */
	public void findShortestPath(Node start, Node goal) {
		// store start and goal node
		this.start = start;
		this.goal = goal;
		// set all node to become not visited
		for (Node node : nodes.values()) {
			node.setVisited(false);
		}
		// declare 'pathFrom' node
		Node pathFrom = null;
		// instantiate a priority queue as a fringe of the paths
		// compare them between shortest path to goal
		PriorityQueue<NodeToNode> paths = new PriorityQueue<NodeToNode>(10,
				toGoalComparator);
		// enqueue start node
		paths.add(new NodeToNode(start, null, 0, estimate(start, goal), null));
		// repeat until fringe is empty
		while (!paths.isEmpty()) {
			NodeToNode path = paths.poll();
			if (!path.node.isVisited()) {
				// give a track to visited node
				path.node.setVisited(true);
				// store node where it comes from
				path.node.setPathFrom(pathFrom);
				// set a cost from origin to this node
				path.node.setCost(path.costToHere);
				// set the edge used to connect this edge to origin edge
				path.node.setEdge(path.edge);
				// check whether the edge is in restrictions
				boolean hasRes = (restrictions.containsKey(path.edge)) ? true
						: false;
				// if node = goal then exit
				if (path.node.equals(goal)) {
					return;
				}
				// iterate through each edge
				// if the end of edge (neighbor) has not been visited yet
				// put it into fringe
				for (Segment s : path.node.getOutNeighbours()) {
					// check whether the second edge will be in restrictions or
					// not. If so, then it should not go through this way
					if (hasRes && restrictions.get(path.edge).contains(s)) {
						continue;
					}
					Node neigh = s.getEndNode();
					if (!neigh.isVisited()) {
						double costToNeigh = path.costToHere + s.getLength();
						double estTotal = costToNeigh + estimate(neigh, goal);
						paths.add(new NodeToNode(neigh, path.node, costToNeigh,
								estTotal, s));
						visitedPaths.add(s);
					}
				}
			}
		}

	}

	/** find the shortest time between two nodes */
	public void findShortestTime(Node start, Node goal) {
		// store start and goal node
		this.start = start;
		this.goal = goal;
		// set all node to become not visited
		for (Node node : nodes.values()) {
			node.setVisited(false);
		}
		// declare 'pathFrom' node
		Node pathFrom = null;
		// instantiate a priority queue as a fringe of the paths
		// compare them between shortest path to goal
		PriorityQueue<NodeToNode> paths = new PriorityQueue<NodeToNode>(10,
				toGoalComparator);
		// calculate initial time cost
		double timeCost = 0;
		// check whether it gets traffic light cost
		if (start.getInNeighbours().size() >= 2
				&& start.getOutNeighbours().size() >= 2) {
			timeCost += TRAFFIC_COST;
		}
		// enqueue start node
		paths.add(new NodeToNode(start, null, timeCost, estimateTime(start,
				goal), null));
		// repeat until fringe is empty
		while (!paths.isEmpty()) {
			NodeToNode path = paths.poll();
			if (!path.node.isVisited()) {
				// give a track to visited node
				path.node.setVisited(true);
				// store node where it comes from
				path.node.setPathFrom(pathFrom);
				// set a cost from origin to this node
				// and check whether it gets delayed by traffic light
				// traffic light usually appear at either t-junction or
				// intersection
				// check whether it gets traffic light cost
				if (path.node.getInNeighbours().size() >= 2
						&& path.node.getOutNeighbours().size() >= 2) {
					path.node.setCost(path.costToHere + TRAFFIC_COST);
				} else {
					path.node.setCost(path.costToHere);
				}

				// check whether 'from' Node has traffic light
				// if so, add the delayed time to the cost
				if (path.from != null) {
					if (path.from.getInNeighbours().size() >= 2
							&& path.from.getOutNeighbours().size() >= 2) {
						visitedPathsAndTimeCosts.put(path.edge,
								visitedPathsAndTimeCosts.get(path.edge)
										+ TRAFFIC_COST);
					}
				}
				// set the edge used to connect this edge to origin edge
				path.node.setEdge(path.edge);
				// check whether the edge is in restrictions
				boolean hasRes = (restrictions.containsKey(path.edge)) ? true
						: false;
				// if node = goal then exit
				if (path.node.equals(goal)) {
					return;
				}
				// iterate through each edge
				// if the end of edge (neighbor) has not been visited yet
				// put it into fringe
				for (Segment s : path.node.getOutNeighbours()) {
					// check whether the second edge will be in restrictions or
					// not. If so, then it should not go through this way
					if (hasRes && restrictions.get(path.edge).contains(s)) {
						continue;
					}
					Node neigh = s.getEndNode();
					if (!neigh.isVisited()) {
						double costToNeigh = path.costToHere + s.getLength()
								/ s.getRoad().getSpeed();
						double estTotal = costToNeigh
								+ estimateTime(neigh, goal) / maxSpeed;
						paths.add(new NodeToNode(neigh, path.node, costToNeigh,
								estTotal, s));
						visitedPathsAndTimeCosts.put(s, s.getLength()
								/ s.getRoad().getSpeed());
					}
				}
			}
		}

	}

	/** method to clear the track of visited path */
	public void clear() {
		visitedPaths = new HashSet<Segment>();
		visitedPathsAndTimeCosts = new HashMap<Segment, Double>();
		articulationPoints = new HashSet<Node>();
		start = null;
		goal = null;
	}

	/** a method to estimate a distance between 2 nodes */
	public double estimate(Node start, Node goal) {
		return start.distanceTo(goal.getLoc());
	}

	/** a method to estimate the time needed to go from one node to another node */
	public double estimateTime(Node start, Node goal) {
		return start.distanceTo(goal.getLoc()) / averageSpeed;
	}

	public static void main(String[] arguments) {
		AucklandMapper.main(arguments);
	}

}
