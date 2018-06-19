package GMS_utils.helper;

import javafx.scene.shape.Polygon;
import org.jgrapht.UndirectedGraph;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;

/**
 * This object holds a graph and the related concave hull of the graph. This saves time while annotating the graph.
 */
public class LarsGraph implements Serializable{
    /**
     * The graph
     */
    private UndirectedGraph<GraphVertex, GraphEdge> graph;

    /**
     * The concave hull of the graph
     */
    private List<PointHD2> concaveHull;

    /**
     * Holds the information if the graph is a annotationgraph
     */
    private boolean annotation = false;

    public LarsGraph(UndirectedGraph<GraphVertex, GraphEdge> graph){
        this(graph, new ArrayList<>());
    }

    private LarsGraph(UndirectedGraph<GraphVertex, GraphEdge> graph, ArrayList<PointHD2> concaveHull){
        this.graph = graph;
        this.concaveHull = concaveHull;
    }

    public LarsGraph(UndirectedGraph<GraphVertex, GraphEdge> graph, boolean annotation){
        this(graph, new ArrayList<>());
        this.annotation = annotation;
    }

    /**
     * Returns the undirected graph
     *
     * @return - the graph
     */
    public synchronized UndirectedGraph<GraphVertex, GraphEdge> getGraph(){
        return graph;
    }

    /**
     * Sets the value of the undirected graph to the given one. Dont forget to recalculate the hull!
     *
     * @param graph - the new graph
     */
    public synchronized void setGraph(UndirectedGraph<GraphVertex, GraphEdge> graph){
        this.graph = graph;
    }

    /**
     * Returns the concave hull of the graph as a list of points
     *
     * @return - concave hull
     */
    public synchronized List<PointHD2> getConcaveHull(){
        return concaveHull;
    }

    /**
     * Sets the concave hull to the given one
     *
     * @param concaveHull - the new hull
     */
    public synchronized void setConcaveHull(List<PointHD2> concaveHull){
        this.concaveHull = concaveHull;
    }

    /**
     * Is this graph an annotation
     *
     * @return
     */
    public boolean isAnnotationGraph(){
        return annotation;
    }

    /**
     * Sets if this graph is a annotation
     *
     * @param annotation
     */
    public void setAnnotationGraph(boolean annotation){
        this.annotation = annotation;
    }

    /**
     * Adds the given edge to the graph
     *
     * @param e - the edge to add
     * @return - if it was successful
     */
    public void addEdge(GraphEdge e, GraphVertex source, GraphVertex target){
        graph.addVertex(source);
        graph.addVertex(target);
        graph.addEdge(source, target, e);
    }

    /**
     * Removes the given edge form the graph
     *
     * @param e - the edge to remove
     * @return - if it was successful
     */
    public void removeEdge(GraphEdge e){
        if(graph.containsEdge(e)){
            graph.removeEdge(e);
        }
    }

    /**
     * Checks if the graph contains the given edge e
     *
     * @param e - edge to check
     * @return - true if the graph contains the edge
     */
    public boolean containsEdge(GraphEdge e){
        return graph.containsEdge(e);
    }


    /**
     * Checks if the given LarsGraph is intersecting with this LarsGraph. This is done based on the hull.
     * The check is done on this LarsGraphs hull and also on the given one.
     *
     * @param lG - LarsGraph to check
     * @return - True if the LarsGraphs intersect else false
     */
    public boolean isIntersectingWith(LarsGraph lG){
        return TopologyUtil.isPolygonInPolygon(concaveHull, lG.asPolygon())
                || TopologyUtil.isPolygonInPolygon(lG.getConcaveHull(), asPolygon());
    }

    /**
     * Checks if the given Polygon is intersecting with this LarsGraph. This is done based on the hull.
     *
     * @param polygon - polygonRepresentation to check
     * @return - True if the LarsGraphs intersect else false
     */
    public boolean isIntersectingWith(Polygon polygon){
        return TopologyUtil.isPolygonInPolygon(concaveHull, polygon);
    }

    /**
     * Checks on of the given LarsGraphs is intersecting with this LarsGraph. This is done based on the hull.
     * The check is done on this LarsGraphs hull and also on the given ones.
     *
     * @param lGs - LarsGraphs to check
     * @return - True if the LarsGraphs intersect else false
     */
    public boolean isIntersectingWith(List<LarsGraph> lGs){
        ListIterator<LarsGraph> iter = lGs.listIterator();
        while(iter.hasNext()){
            LarsGraph lG = iter.next();
            if(TopologyUtil.isPolygonInPolygon(concaveHull, lG.asPolygon())
                    || TopologyUtil.isPolygonInPolygon(lG.getConcaveHull(), asPolygon())){
                return true;
            }
        }

        return false;
    }

    /**
     * Returns the polygon representation of the current graph
     *
     * @return - the graph as polygon
     */
    private Polygon asPolygon(){
        Polygon p = new Polygon();
        for(GraphVertex v : graph.vertexSet()){
            p.getPoints().add(v.x());
            p.getPoints().add(v.y());
        }

        return p;
    }
}
