package aucklandMapper;
/* Code for COMP102 Assignment
 * Name:
 * Usercode:
 * ID:
 */

import java.util.*;


/** Road   */

public class Road{

    private int id;  
    private String name;  
    private String city;  
    private boolean oneway;  
    private int speed;  
    private int roadClass;  
    private boolean notForCars;  
    private boolean notForPedestrians;  
    private boolean notForBicycles;  
    private List<Segment> segments = new ArrayList<Segment>();  

    /** Construct a new Road object */
    public Road(int id, String nm, boolean ow, int sp, int rc, boolean nCars, boolean nPed, boolean nBic){
	this.id = id;  
	name = nm;  
	oneway = ow;  
	speed = sp;  
	roadClass = rc;  
	notForCars = nCars;  
	notForPedestrians = nPed;  
	notForBicycles = nBic;  
    }

    /** Construct a new Road object from a line from the data file*/
    public Road(String line){
	String[] values = line.split("\t");
	id = Integer.parseInt(values[0]);  //id
	name = values[2];
	city = values[3];
	if (city.equals("-")) { city = ""; }
	oneway = values[4].equals("1");  
	speed = Integer.parseInt(values[5]);
	roadClass = Integer.parseInt(values[6]);  
	notForCars = values[7].equals("1");  
	notForPedestrians = values[8].equals("1");  
	notForBicycles = values[9].equals("1");  
    }


    public int getID(){return this.id;}
    public String getName(){return this.name;}
    public String getFullName(){
	if (this.city=="") {return this.name;}
	return this.name + " " + this.city;
    }
    public int getRoadclass(){return this.roadClass;}
    public int getSpeed(){return this.speed;}

    public boolean isOneWay(){return this.oneway;}
    public boolean isNotForCars(){return this.notForCars;}
    public boolean isNotForPedestrians(){return this.notForPedestrians;}
    public boolean isNotForBicycles(){return this.notForBicycles;}

    public void addSegment(Segment seg){
	this.segments.add(seg);
    }

    public List<Segment> getSegments(){
	return this.segments;
    }

    public String toString(){
	return "Road: "+getFullName();
    }

    public static void main(String[] arguments){
	AucklandMapper.main(arguments);
    }	


}
