package aucklandMapper;

/* Code for COMP261 Assignment
 */

// call repaint() on this object to invoke the drawing.

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Point;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.filechooser.FileFilter;

public class AucklandMapper {

	private JFrame frame;
	private JComponent drawing;
	private JTextArea textOutput;
	private JTextField nameEntry;
	private int windowSize = 700;

	private RoadGraph roadGraph;

	private Node selectedNode; // the currently selected node
	private List<Segment> selectedSegments; // the currently selected road or
											// path

	private boolean showArtPoints;
	private boolean findShortestDist;
	private boolean findShortestTime;
	private Node start;
	private Node goal;

	private boolean loaded = false;

	// Dimensions for drawing
	double westBoundary;
	double eastBoundary;
	double southBoundary;
	double northBoundary;
	Location origin;
	double scale;

	public AucklandMapper(String dataDir) {
		setupInterface();
		roadGraph = new RoadGraph();

		textOutput.setText("Loading data...");
		while (dataDir == null) {
			dataDir = getDataDir();
		}
		textOutput.append("Loading from " + dataDir + "\n");
		textOutput.append(roadGraph.loadData(dataDir));
		setupScaling();
		loaded = true;
		drawing.repaint();
	}

	private class DirectoryFileFilter extends FileFilter {
		public boolean accept(File f) {
			return f.isDirectory();
		}

		public String getDescription() {
			return "Directories only";
		}
	}

	private String getDataDir() {
		JFileChooser fc = new JFileChooser();
		fc.setFileFilter(new DirectoryFileFilter());
		fc.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
		if (fc.showOpenDialog(frame) != JFileChooser.APPROVE_OPTION) {
			return null;
		}
		return fc.getSelectedFile().getPath() + File.separator;
	}

	private void setupScaling() {
		double[] b = roadGraph.getBoundaries();
		westBoundary = b[0];
		eastBoundary = b[1];
		southBoundary = b[2];
		northBoundary = b[3];
		resetOrigin();
		/*
		 * System.out.printf("Boundaries: w %.2f, e %.2f, s %.2f, n %.2f%n",
		 * b[0], b[1], b[2], b[3]);
		 * System.out.printf("Scaling from %s @ %.5f,%n", origin, scale);
		 */
	}

	private void setupInterface() {
		// Set up a window .
		frame = new JFrame("Graphics Example");
		frame.setSize(windowSize + 100, windowSize);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		// Set up a JComponent in the window that we can draw on
		// When the JComponent tries to paint itself, it will call the redraw
		// method
		// in this PathDrawer class, passing a Graphics object to it.
		// The redraw method can draw whatever it wants on the Graphics object.
		// We can ask the JComponent to paint itself by calling
		// drawing.repaint()
		// Note that this merely requests that the drawing is repainted; it
		// won't
		// necessarily do it immediately.
		drawing = new JComponent() {
			protected void paintComponent(Graphics g) {
				redraw(g);
			}
		};
		frame.add(drawing, BorderLayout.CENTER);

		// Setup a text area for output
		textOutput = new JTextArea(5, 100);
		textOutput.setEditable(false);
		JScrollPane textSP = new JScrollPane(textOutput);
		frame.add(textSP, BorderLayout.SOUTH);

		// Set up a panel for some buttons.
		// To get nicer layout, we would need a LayoutManager on the panel.
		JPanel panel = new JPanel();
		frame.add(panel, BorderLayout.NORTH);

		// Add a text label to the panel.
		// panel.add(new JLabel("Click to select"));

		// Add buttons to the panel.
		JButton button = new JButton("+");
		panel.add(button);
		button.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent ev) {
				zoomIn();
			}
		});

		button = new JButton("-");
		panel.add(button);
		button.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent ev) {
				zoomOut();
			}
		});

		button = new JButton("<");
		panel.add(button);
		button.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent ev) {
				pan("left");
			}
		});

		button = new JButton(">");
		panel.add(button);
		button.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent ev) {
				pan("right");
			}
		});

		button = new JButton("^");
		panel.add(button);
		button.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent ev) {
				pan("up");
			}
		});

		button = new JButton("v");
		panel.add(button);
		button.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent ev) {
				pan("down");
			}
		});

		// articulation points
		button = new JButton("Articulation Points");
		panel.add(button);
		button.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent ev) {
				clear();
				if (!showArtPoints) {
					roadGraph.findArticulationPoints();
					showArtPoints = true;
				} else {
					showArtPoints = false;
				}
				drawing.repaint();
			}
		});

		// shortest distance
		button = new JButton("Shortest By Distance");
		panel.add(button);
		button.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent ev) {
				clear();
				findShortestDist = true;
				findShortestTime = false;
				drawing.repaint();
			}
		});

		// shortest distance
		button = new JButton("Shortest By Time");
		panel.add(button);
		button.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent ev) {
				clear();
				findShortestTime = true;
				findShortestDist = false;
				drawing.repaint();
			}
		});

		// Node details
		button = new JButton("Node Details");
		panel.add(button);
		button.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent ev) {
				clear();
				findShortestDist = false;
				findShortestTime = false;
				drawing.repaint();
			}
		});

		nameEntry = new JTextField(20);
		panel.add(nameEntry);
		nameEntry.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				lookupName(nameEntry.getText());
				drawing.repaint();
			}
		});

		button = new JButton("Quit");
		panel.add(button);
		button.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent ev) {
				System.exit(0);
			}
		});

		// Add a mouselistener to the drawing JComponent to respond to mouse
		// clicks.
		drawing.addMouseListener(new MouseAdapter() {
			public void mouseReleased(MouseEvent e) {
				if (findShortestDist) {
					if (start == null) {
						start = findNode(e.getPoint());
					} else if (goal == null) {
						goal = findNode(e.getPoint());
						if (goal.equals(start)) {
							goal = null;
							return;
						}
						if (goal != null) {
							findShortestDist = false;
							roadGraph.findShortestPath(start, goal);
							textOutput
									.setText("Past roads from start to goal : \n");
							for (String s : roadGraph.roadsGoThroughByDist()) {
								textOutput.append(s + "\n");
							}
						}
					}
				} else if (findShortestTime) {
					if (start == null) {
						start = findNode(e.getPoint());
					} else if (goal == null) {
						goal = findNode(e.getPoint());
						if (goal.equals(start)) {
							goal = null;
							return;
						}
						if (goal != null) {
							findShortestTime = false;
							roadGraph.findShortestTime(start, goal);
							textOutput
									.setText("Past roads from start to goal : \n");
							for (String s : roadGraph.roadsGoThroughByTime()) {
								textOutput.append(s + "\n");
							}
						}
					}
				} else {
					selectedNode = findNode(e.getPoint());
					if (selectedNode != null) {
						textOutput.setText(selectedNode.toString());
					}
				}
				drawing.repaint();
			}
		});

		// Once it is all set up, make the interface visible
		frame.setVisible(true);

	}

	private double zoomFactor = 1.25;
	private double panFraction = 0.2;

	// set origin and scale for the whole map
	private void resetOrigin() {
		origin = new Location(westBoundary, northBoundary);
		scale = Math.min(windowSize / (eastBoundary - westBoundary), windowSize
				/ (northBoundary - southBoundary));
	}

	// shrink the scale (pixels/per km) by zoomFactor and move origin
	private void zoomOut() {
		scale = scale / zoomFactor;
		double deltaOrig = windowSize / scale * (zoomFactor - 1) / zoomFactor
				/ 2;
		origin = new Location(origin.x - deltaOrig, origin.y + deltaOrig);
		drawing.repaint();
	}

	// expand the scale (pixels/per km) by zoomFactor and move origin
	private void zoomIn() {
		double deltaOrig = windowSize / scale * (zoomFactor - 1) / zoomFactor
				/ 2;
		origin = new Location(origin.x + deltaOrig, origin.y - deltaOrig);
		scale = scale * zoomFactor;
		drawing.repaint();
	}

	private void pan(String dir) {
		double delta = windowSize * panFraction / scale;
		switch (dir) {
		case "left": {
			origin = new Location(origin.x - delta, origin.y);
			break;
		}
		case "right": {
			origin = new Location(origin.x + delta, origin.y);
			break;
		}
		case "up": {
			origin = new Location(origin.x, origin.y + delta);
			break;
		}
		case "down": {
			origin = new Location(origin.x, origin.y - delta);
			break;
		}
		}
		drawing.repaint();
	}

	// Find the place that the mouse was clicked on (if any)
	private Node findNode(Point mouse) {
		return roadGraph.findNode(mouse, origin, scale);
	}

	private void lookupName(String query) {
		List<String> names = new ArrayList(roadGraph.lookupName(query));
		if (names.isEmpty()) {
			selectedSegments = null;
			textOutput.setText("Not found");
		} else if (names.size() == 1) {
			String fullName = names.get(0);
			nameEntry.setText(fullName);
			textOutput.setText("Found");
			selectedSegments = roadGraph.getRoadSegments(fullName);
		} else {
			selectedSegments = null;
			String prefix = maxCommonPrefix(query, names);
			nameEntry.setText(prefix);
			textOutput.setText("Options: ");
			for (int i = 0; i < 10 && i < names.size(); i++) {
				textOutput.append(names.get(i));
				textOutput.append(", ");
			}
			if (names.size() > 10) {
				textOutput.append("...\n");
			} else {
				textOutput.append("\n");
			}
		}
	}

	private String maxCommonPrefix(String query, List<String> names) {
		String ans = query;
		for (int i = query.length();; i++) {
			if (names.get(0).length() < i)
				return ans;
			String cand = names.get(0).substring(0, i);
			for (String name : names) {
				if (name.length() < i)
					return ans;
				if (name.charAt(i - 1) != cand.charAt(i - 1))
					return ans;
			}
			ans = cand;
		}
	}

	// The redraw method that will be called from the drawing JComponent and
	// will
	// draw the map at the current scale and shift.
	public void redraw(Graphics g) {
		if (roadGraph != null && loaded) {
			roadGraph.redraw(g, origin, scale);

			// finding shortest path
			if (start != null) {
				g.setColor(Color.red);
				start.draw(g, origin, scale);
			}
			if (goal != null) {
				g.setColor(Color.red);
				goal.draw(g, origin, scale);
			}

			// node details
			if (selectedNode != null) {
				g.setColor(Color.red);
				selectedNode.draw(g, origin, scale);
			}
			if (selectedSegments != null) {
				g.setColor(Color.red);
				for (Segment seg : selectedSegments) {
					seg.draw(g, origin, scale);
				}
			}
		}
	}

	/** method that clear track of nodes */
	public void clear() {
		start = null;
		goal = null;
		selectedNode = null;
		findShortestDist = false;
		findShortestTime = false;
		roadGraph.clear();
	}

	public static void main(String[] arguments) {
		if (arguments.length > 0) {
			AucklandMapper obj = new AucklandMapper(arguments[0]);
		} else {
			AucklandMapper obj = new AucklandMapper(null);
		}
	}

}
