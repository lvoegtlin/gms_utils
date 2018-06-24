package ch.unifr.GMS_utils.graph;


import com.google.common.collect.TreeMultimap;
import com.vividsolutions.jts.geom.Coordinate;
import org.openimaj.math.geometry.point.Point2d;
import org.openimaj.math.geometry.point.Point2dImpl;

import java.awt.*;
import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * HisDoc2Java
 * Package: ch.unifr.hisdoc2.interestPoints
 * Date: 17/07/13 08:46
 * <p>
 * An Interest Point class which encapsulates different Point classes of different libraries, which are needed in the Project
 * <p>
 * Compatibility to java.awt.Point, org.apache.commons.math3.ml.clustering.DoublePoint, georegression.struct.point.Point2D_F64
 */

public class PointHD2 extends Point2dImpl implements Point2d, Serializable, Comparable, Cloneable {
    /**
     * Serializable version identifier.
     */
    private static final long serialVersionUID = 3946024775784901369L;
    private static final float DEFAULT_VALUE = Float.NEGATIVE_INFINITY;

    /**
     * ID of the connected component the point is part of
     */
    private int connectedComponentID = Integer.MAX_VALUE;
    /**
     * ID of the connected component the point is part of
     */
    private int graphccID = Integer.MAX_VALUE;

    private double distance = 0;


    /**
     * empty constructor
     */
    protected PointHD2() {
        super();
        this.x = DEFAULT_VALUE;
        this.y = DEFAULT_VALUE;
    }

    public PointHD2(Point2d point2d, int ccID) {
        super(point2d);
        this.connectedComponentID = ccID;
    }

    public PointHD2(PointHD2 pt) {
        this.x = pt.x;
        this.y = pt.y;
        this.connectedComponentID = pt.connectedComponentID;
    }

    public PointHD2(Point2d pt) {
        this.x = pt.getX();
        this.y = pt.getY();
    }

    /**
     * Create an PointHD2 from two doubles
     *
     * @param x coordinate
     * @param y coordinate
     */
    protected PointHD2(final float x, final float y) {
        this.set(x, y);
    }


    /**
     * Create an PointHD2 from two doubles
     *
     * @param x coordinate
     * @param y coordinate
     */
    public PointHD2(final double x, final double y) {
        this.set((float)x, (float)y);
    }

    /**
     * Create an PointHD2 from a java.awt.Point
     *
     * @param pt Point which will be transformed
     */
    public PointHD2(final Point pt) {
        super();
        this.set(pt.x, pt.y);
    }


    /*********************** own ************************/
    public PointHD2(final Point2D pt) {
        super();
        this.set((float) pt.getX(), (float) pt.getY());
    }

    /**
     * Create an PointHD2 from a two-dimensional double array
     *
     * @param point the 2-dimensional point in double space
     */
    public PointHD2(final float[] point) {
        super();
        this.set(point[0], point[1]);
    }


    /**
     * Create an PointHD2 from a two-dimensional double array
     *
     * @param point the 2-dimensional point in double space
     */
    public PointHD2(final double[] point) {
        super();
        this.set((float) point[0], (float) point[1]);
    }

    /**
     * @param point the 2-dimensional point in integer space
     */
    public PointHD2(final int[] point) {
        super();
        this.set((float) point[0], (float) point[1]);
    }

    public static List<PointHD2> coordinateList2pointList(List<Coordinate> coordinates) {
        List<PointHD2> points = new ArrayList<>(coordinates.size());
        for (Coordinate c : coordinates) {
            points.add(new PointHD2((float) c.x, (float) c.y));
        }
        return points;
    }

    public static <P extends PointHD2> List<Coordinate> pointList2coordinateList(List<P> points) {
        List<Coordinate> coordinates = new ArrayList<>(points.size());
        for (PointHD2 p : points) {
            coordinates.add(p.toCoordinate());
        }
        return coordinates;
    }

    public static <P extends PointHD2> List<Point2d> toPoint2dOpenImaJList(List<P> points) {
        List<Point2d> pts = new ArrayList<>(points.size());

        for (PointHD2 p : points) {
            pts.add(p.toPoint2dOpenImaJ());
        }
        return pts;
    }

    private void set(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public PointHD2 createNewInstance() {
        return new PointHD2();
    }

    public double[] getPoint() {
        return new double[]{this.x, this.y};
    }

    public boolean equals(final Object other) {
        return other instanceof PointHD2 && this.x == ((PointHD2) other).x && this.y == ((PointHD2) other).y;
    }

    /**
     * {@inheritDoc}
     */
    public String toString() {
        return "P( " + String.format("%.2f", this.x) + ", " + String.format("%.2f", this.y) + " )";
    }

    public PointHD2 plus(final PointHD2 p) {
        PointHD2 n = new PointHD2(this.x, this.y);
        n.x += p.x;
        n.y += p.y;
        return n;
    }

    public PointHD2 minus(final PointHD2 p) {
        PointHD2 n = new PointHD2(this.x, this.y);
        n.x -= p.x;
        n.y -= p.y;
        return n;
    }

    public double euclideanDistance(final PointHD2 point) {
        float x = this.x - point.x;
        float y = this.y - point.y;
        return Math.sqrt(x*x + y*y);
    }

    public PointHD2 distanceVector(final PointHD2 p) {
        return new PointHD2(p.x - x, p.y - y);
    }

    /**
     * vector norm - assume the point is a vector from the origin of the coordinate system.
     *
     * @return vector norm (length of the vector): sqrt(x^2+y^2)
     */
    public double norm() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    /**
     * transform the PointHD2 into a java.awt.Point
     *
     * @return coordinates as point
     */
    public Point toAWTPoint() {
        return new Point((int) this.x, (int) this.y);
    }

    /**
     * transform the PointHD2 into a java.awt.Point2D
     *
     * @return coordinates as point
     */
    public Point2D toPoint2D() {
        return new Point2D.Float(this.x, this.y);
    }

    /**
     * transform the PointHD2 into a double array - needed for clustering
     *
     * @return coordinates as double array
     */
    public double[] toArray() { return new double[]{this.x, this.y}; }

    /**
     * Compares this object with the specified object for order.  Returns a
     * negative integer, zero, or a positive integer as this object is less
     * than, equal to, or greater than the specified object.
     *
     * @param o the object to be compared.
     * @return a negative integer, zero, or a positive integer as this object
     * is less than, equal to, or greater than the specified object.
     * @throws NullPointerException if the specified object is null
     * @throws ClassCastException   if the specified object's type prevents it
     *                              from being compared to this object.
     */
    @Override
    public int compareTo(final Object o) throws NullPointerException, ClassCastException {
        PointHD2 p = (PointHD2) o;
        return Float.compare(this.x, p.x);
    }

    public double x() {return this.x;}

    public double y() { return this.y; }

    /**
     * Determine whether a point lies on one side of a line or on the other using the vector cross product.
     * The orientation of the line matters (i.e., AB != BA)
     * <p>
     * The cross product v x w of two vectors v and w is a vector whose length is |v||w|sin(phi) (where |v| is the
     * length of v and phi is the angle between the vectors), and who is orthogonal (perpendicular) to both v and w.
     * <p>
     * Since there are two such possible vectors, the definition arbitrarily selects the one that matches the
     * direction in which a screw would move if rotated from v to w
     * <p>
     * Mathematically, if the coordinates of vectors v and w are (v_x,v_y) and (w_x,w_y) respectively, the cross produt
     * will be (v_x w_y - w_y w_x). So now, if you have a segment defined by points A B and want to check on which side
     * of AB a third point P falls, you only need to compute the cross product AB x AP and check its sign: If it's
     * negative, it will be on the "right" side of AB (when standing on A and looking towards B). If positive, it will
     * be on the left side.
     *
     * @param A line start
     * @param B line end
     * @return 1 if "left", -1 if "right"
     */
    public int pointLocationAsRegardsLine(final PointHD2 A, final PointHD2 B) {
        double cp1 = (B.x - A.x) * (this.y - A.y) - (B.y - A.y) * (this.x - A.x);
        return (cp1 > 0) ? 1 : -1;
    }

    /**
     * Calculates a fast "pseudo distance" between this point and line AB for the purpose of comparing distances
     * <p>
     * When trying to find the point P that is farthest away from AB, the segment AB never changes.
     * When comparing distances, the exact distance value is not needed. Thus, we do not need to divide by
     * the length of the AB segment, or take the square root.
     * <p>
     * http://www.ahristov.com/tutorial/geometry-games/convex-hull.html
     *
     * @param A line start
     * @param B line end
     * @return pseudo distance
     */
    public double fastpointToLinePseudoDistance(final PointHD2 A, final PointHD2 B) {
        double ABx = B.x - A.x;
        double ABy = B.y - A.y;
        double num = ABx * (A.y - this.y) - ABy * (A.x - this.x);

        return Math.abs(num);
    }

    /**
     * Calculates the distance between this point and line AB
     * The distance between a point P and a line AB is the length of the segment that crosses the line, is perpendicular
     * to it, and passes through P
     * <p>
     * To compute it, assume that we have the locations of two points on the line: (x1,y1) and (x2,y2).
     * <p>
     * http://www.ahristov.com/tutoriales/geometry-games/point-line-distance.html
     *
     * @param A line start
     * @param B line end
     * @return distance
     */
    public double pointToLineDistance(PointHD2 A, PointHD2 B) {
        double ABx = B.x - A.x;
        double ABy = B.y - A.y;

        double normalLength = Math.sqrt(ABx * ABx + ABy * ABy);

        return Math.abs(ABy * (this.x - A.x) - ABx * (this.y - A.y)) / normalLength;
    }

    public boolean isInitialized() {
        return x != DEFAULT_VALUE && y != DEFAULT_VALUE;
    }

    /**
     * brute force knn
     *
     * @param set list of points to search neighbors in
     * @param k   number of neighbors
     * @return list of k nearest neighbors
     */
    public List<PointHD2> knn(List<PointHD2> set, int k) {

        TreeMultimap<Double, PointHD2> neighbors = TreeMultimap.create();

        for (PointHD2 p : set)
            neighbors.put(this.euclideanDistance(p), p);
        List<PointHD2> knn = new ArrayList<>(k);
        Object[] pts = neighbors.values().toArray();

        for (int i = 0; i < Math.min(k, neighbors.size()); i++)
            knn.add((PointHD2) pts[i]);


        return knn;
    }

    public double angle() {
        return Math.atan2(y, x);
    }

    public Coordinate toCoordinate() {
        return new Coordinate(x, y);
    }

    private Point2d toPoint2dOpenImaJ() {
        return new Point2dImpl((float) x(), (float) y());
    }

    public PointHD2 matrixMultiplication(float[][] m) {
        PointHD2 np = this.clone();
        np.x = m[0][0] * x + m[1][0] * y;
        np.y = m[0][1] * x + m[1][1] * y;
        return np;
    }

    public int getConnectedComponentID() {
        return connectedComponentID;
    }

    public PointHD2 clone() {
        PointHD2 newPoint = new PointHD2(super.clone());
        newPoint.connectedComponentID = this.connectedComponentID;

        return newPoint;
    }
    
    @Override
    public int hashCode() {
        return Float.hashCode(x)+Float.hashCode(y);
    }

    public void setccID(int ccID) {
        connectedComponentID = ccID;
    }

    public void setgraphccID(int ccID) {
        graphccID = ccID;
    }

    public int graphccID() {
        return graphccID;
    }

    public double getDistance(){
        return distance;
    }

    public void setDistance(double distance){
        this.distance = distance;
    }
}


