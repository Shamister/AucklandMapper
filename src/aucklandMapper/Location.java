package aucklandMapper;
/* Code for COMP261 Assignment
 */

import java.awt.Color;
import java.util.*;
import java.io.*;
import java.awt.Point;


/** Location
    A Location is a point in a 2D coordinate system,
    with increasing x from west to east and increasing y
    from south to north (ordinary mathematical coordinates).
    Locations are represented with two doubles, (with an unspecified
    length unit - could be kilometers, for example).

    Location provides methods for converting between Locations
    and Points representing pixel positions on a window/screen.
    A Point is represented by two integers representing screen/window
    positions (x to the right and y down).

    The conversion needs
    - an origin Location [a Location at the origin will be converted to
      the point (0,0)]
    - a scale specifying how many pixels per length unit.
      Typically the scale will be  ( windowSize /(maxLocation - minLocation))
   */

public class Location{

    // fields are public for easy access.
    // they are final so that the location is immutable
    public final double x;
    public final double y;

    /**
       Construct a new Location object
    */
    public Location(double x, double y){
	this.x = x;
	this.y = y;
    }

    // ---------   CONVERSION methods ------------------------
    /**
       Return a location specified by a point on the window, 
       given an origin and a scale. Note the vertical direction is inverted.
       invoked as Location.newFromPoint(point, orig, scale);
    */
    public static Location newFromPoint(Point point, Location origin, double scale){
	return new Location(point.x/scale + origin.x, origin.y - point.y/scale);
    }

    /**
       Return the point on the window corresponding to a location, given an
       given an origin and a scale. Note the vertical direction is inverted
    */
    public Point getPoint(Location origin, double scale){
	int u = (int)((x-origin.x)*scale);
	int v = (int)((origin.y-y)*scale);
	return new Point(u, v);
    }

    private static final double centerLat = -36.847622;  // the center of Auckland City!
    private static final double centerLon = 174.763444;  // according to Google Maps
    /**
       Return a location specified by a latitude and a longitude
    */
    private static final double scaleLat = 111.0;    // how many kilometers per degree.
    private static final double degToRad = Math.PI/180;
    public static Location newFromLatLon(double lat, double lon){
	double y = (lat - centerLat) *scaleLat;
	double x = (lon - centerLon) * (scaleLat * Math.cos((lat-centerLat)*degToRad));
	return new Location(x, y);
    }

    // ---------   UTILITY methods on Locations ------------------------
    /**
       Return distance between this location and another
     */
    public double distanceTo(Location other){
	return Math.hypot(this.x-other.x, this.y-other.y);
    }

    /**
       Return true if this location is within dist of other
       Uses manhattan distance for greater speed.
       Equivalent to whether other is within a diamond shape around this location.
       
    */
    public boolean closeTo(Location other, double dist){
	return Math.abs(this.x-other.x)+Math.abs(this.y-other.y)<= dist;
    }

    public String toString(){
	return String.format("(%.5f, %.5f)", x, y);
    }


    /** Test method */
    public static void main(String[] arguments){
	System.out.println("new Location(10.5, 42.0) -> " + new Location(10.5, 42.0));

	// The boundaries on the  coords of the locations:
	double west = 20.0;   
	double east = 120.0;
	double north = 200.0;   
	double south = 100.0;
	
	// the origin of the window corresponds to location (west, north)=(20.0, 200.0)
	Location origin = new Location(west, north);
	// the scale is 500 pixels for the full width (east - west)
	double scale = 500/(east-west);

	// work out the pixel position where a location should be drawn
	System.out.println("\n*** Converting from Location to Point");

	Location place = new Location(30.0, 150.0);

	System.out.printf("%norigin: loc:%s%nscale: %.3f%nplace: %s%n",
			  origin, scale, place);
	System.out.printf("%nplace.getPoint(orig, scale) -> %s%n",
			  place.getPoint(origin, scale));
	

	// work out the location corresponding to a mouse click
	System.out.println("\n*** Converting from Point to Location");
	Point mouse = new Point(100, 250);

	System.out.printf("%norigin: loc:%s%nscale: %.3f%nmouse: %s%n",
			  origin, scale, mouse);
	
	// work out which location this corresponds to:
	System.out.printf("%nLocation.newFromPoint(mouse, origin, scale) -> %s%n",
			  Location.newFromPoint(mouse, origin, scale));

	// work out location from lat/long:
	System.out.printf("%nLocation.newFromLatLon(-37.8899, 174.724) -> loc:%s%n",
			  Location.newFromLatLon(-37.8899, 174.724));
    }	



}
