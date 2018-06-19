package GMS_utils.helper;

import org.jgrapht.UndirectedGraph;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * This object holds a graph and the related concave hull of the graph. This saves time while annotating the graph.
 */
public class LarsGraphCollection implements Serializable{
    /**
     * The graph
     */
    private List<LarsGraph> graphs;

    /**
     * The concave hull of all the graphs
     */
    private List<PointHD2> concaveHull;

    /**
     * Tells if the LarsGraphCollection is annotated. Thats the case if it has a annotation graph
     */
    private boolean annotated = false;

    /**
     * last graph that was edited
     */
    private LarsGraph editedGraph;

    /**
     * The annotation Graphs of this LGC
     */
    private List<LarsGraph> annotationGraphs;
    /**
     * The non annotation graphs of this LGC
     */
    private List<LarsGraph> nonAnnotationGraphs;

    public LarsGraphCollection(LarsGraph graph){
        this(graph, new ArrayList<>());
    }

    public LarsGraphCollection(LarsGraph graph, List<PointHD2> concaveHull){
        this.graphs = new ArrayList<>();
        this.concaveHull = concaveHull;
        this.annotationGraphs = new ArrayList<>();
        this.nonAnnotationGraphs = new ArrayList<>();

        if(graph != null){
            graphs.add(graph);
            editedGraph = graph;
            if(graph.isAnnotationGraph()){
                annotationGraphs.add(graph);
            } else {
                nonAnnotationGraphs.add(graph);
            }
        }

        update();
    }


    /**
     * Returns the undirected graph
     *
     * @return - the graph
     */
    public synchronized List<LarsGraph> getGraphs(){
        return graphs;
    }

    /**
     * Sets the value of the undirected graph to the given one. Dont forget to recalculate the hull!
     *
     * @param graphs - the new graphs
     */
    public synchronized void setGraphs(List<LarsGraph> graphs){
        this.graphs = new ArrayList<>();
        this.nonAnnotationGraphs = new ArrayList<>();
        this.annotationGraphs = new ArrayList<>();

        LarsGraph[] graphsArray = new LarsGraph[graphs.size()];
        addGraph(graphs.toArray(graphsArray));

        annotationCheck();
        updateHull();
    }

    public LarsGraph getEditedGraph(){
        return editedGraph;
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
     * Is the graph already annotated
     *
     * @return
     */
    public boolean isAnnotated(){
        return annotated;
    }

    /**
     * Returns the annotationsGraphs from this LGC
     *
     * @return - just annotationGraphs
     */
    public List<LarsGraph> getAnnotationGraphs(){
        return annotationGraphs;
    }

    public List<LarsGraph> getNonAnnotationGraphs(){
        return nonAnnotationGraphs;
    }

    public void setAnnotated(boolean status){
        annotated = status;
    }

    /**
     * Returns the LarsGraph that contains a certain GraphVertex. If non of the nonAnnotationGraphs
     * is containing the Vertex it will return null.
     *
     * @param v1 - Vertex we want the graph of
     * @param v2 - Vertex we want the graph of
     * @return - the graph where v is member of or null
     */
    public LarsGraph getLarsGraphByVertex(GraphVertex v1, GraphVertex v2){
        for(LarsGraph lG : nonAnnotationGraphs){
            if(lG.getGraph().containsVertex(v1) || lG.getGraph().containsVertex(v2)){
                return lG;
            }
        }

        return null;
    }

    /**
     * Adds the given edge to the graph
     *
     * @param e - the edge to add
     */
    public void addEdge(GraphEdge e, GraphVertex source, GraphVertex target){
        UndirectedGraph<GraphVertex, GraphEdge> g = editedGraph.getGraph();
        g.addVertex(source);
        g.addVertex(target);
        g.addEdge(source, target, e);
    }

    /**
     * Removes the given edge form the graph
     *
     * @param e - the edge to remove
     */
    public void removeEdge(GraphEdge e){
        for(LarsGraph g : graphs){
            if(g.containsEdge(e)){
                g.removeEdge(e);
                editedGraph = g;
            }
        }
    }

    /**
     * adds a graph to the graphs list. Dont forget to recalculate the hull!
     *
     * @param larsGraphs
     */
    public void addGraph(LarsGraph... larsGraphs){
        for(LarsGraph graph : larsGraphs){
            if(!this.graphs.contains(graph)){
                this.graphs.add(graph);

                if(graph.isAnnotationGraph()){
                    annotationGraphs.add(graph);
                } else {
                    nonAnnotationGraphs.add(graph);
                }

                editedGraph = graph;
            }

            annotationCheck();
        }
    }

    /**
     * Removes a given graph out of the graph list. If the graph is annotation or non-annoation graph it also deletes
     * it out of these lists.
     *
     * @param graphsToRemove - the lG to remove
     */
    public void removeGraph(LarsGraph... graphsToRemove){
        for(LarsGraph graph : graphsToRemove){
            graphs.remove(graph);
            if(graph.isAnnotationGraph()){
                annotationGraphs.remove(graph);
            } else {
                nonAnnotationGraphs.remove(graph);
            }
        }
    }

    /**
     * Updates the hull and the vertices
     */
    public void update(){
        if(graphs.size() == 0){
            concaveHull.clear();
        } else {
            updateHull();
            annotationCheck();
        }
    }

    /**
     * Updates the hull of this collection.
     * It unites all the hulls of the lasrGraphs in the graphs list
     */
    public void updateHull(){
        List<List<PointHD2>> hulls = new ArrayList<>();
        graphs.forEach(larsGraph -> hulls.add(larsGraph.getConcaveHull()));

        try{
            concaveHull = TopologyUtil.getUnionOfHulls(hulls);
        } catch(IllegalArgumentException e){
            e.printStackTrace();
        }

        //TODO start a hull calc service to make a nicer hull
    }

    /**
     * Checks if the collection is annotated or not.
     * This is done by checking if each graph in the graphs list is annotated. If so its true else false.
     */
    private void annotationCheck(){
        if(annotationGraphs.size() != 0){
            annotated = true;
            return;
        }

        annotated = false;
    }

    /**
     * Checks if the graph contains the given edge e
     *
     * @param e         - edge to check
     * @param allGraphs - true if we want all graphs else just the nonannotation graphs
     * @return - true if the graph contains the edge
     */
    public boolean containsEdge(GraphEdge e, boolean allGraphs){
        List<LarsGraph> graphList;
        if(allGraphs){
            graphList = graphs;
        } else {
            graphList = nonAnnotationGraphs;
        }
        for(LarsGraph g : graphList){
            if(g.containsEdge(e)){
                return true;
            }
        }

        return false;
    }

}
