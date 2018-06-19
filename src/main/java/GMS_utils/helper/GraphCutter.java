package GMS_utils.helper;

import org.apache.commons.lang.time.StopWatch;
import org.jgrapht.Graph;
import org.jgrapht.graph.SimpleWeightedGraph;
import org.jgrapht.graph.Subgraph;

import java.util.*;


/**
 * HisDoc2Java
 * Package: ch.unifr.hisdoc2.graphProcessing
 * Date: 29.04.15 4:43 PM
 */
public class GraphCutter{

    private BinClass[] currentHistorgram;
    private Subgraph<GraphVertex, GraphEdge, SimpleWeightedGraph<GraphVertex, GraphEdge>> graph;

    public GraphCutter(Subgraph<GraphVertex, GraphEdge, SimpleWeightedGraph<GraphVertex, GraphEdge>> mstGraph){
        this.graph = mstGraph;
        this.currentHistorgram = createHistogram();
    }

    /**
     * Cuts the heaviest Edges of a list of larsGraphs.
     *
     * @param graphs
     */
    public void cutHighCostEdges(ArrayList<LarsGraph> graphs, double pro){
        for(LarsGraph lG : graphs){
            cutHighCostEdges(lG.getGraph(), pro);
        }
    }

    /**
     * cut those edge classes which summed up are less the the threshold.
     */
    public void cutHighCostEdges(Graph g, double pro){
        StopWatch sw = new StopWatch();
        sw.start();
        int nb = 0;
        double sum = 0;
        double threshold = pro;

        double classSum = 0;
        //The classes with the heaviest edges are at the end of the array
        for(BinClass c : currentHistorgram){
            if(classSum > threshold){
                break;
            }

            for(GraphEdge e : c.edges){
                if(g.containsEdge(e)){
                    e.setDeleted(true);
                    g.removeEdge(e);
                    nb++;
                    sum += Math.abs(graph.getEdgeWeight(e));
                    classSum += c.procent;
                }

            }
        }

        sw.stop();

        System.out.println(nb + " edges were cut due to their high cost, their sum was " + sum);
    }

    /**
     * get the list of edges of the graph sorted by their weight
     *
     * @param graph
     * @param edges
     * @return
     */
    private ArrayList<GraphEdge> getSortedEdges(final Subgraph<GraphVertex, GraphEdge, SimpleWeightedGraph<GraphVertex, GraphEdge>> graph, Set<GraphEdge> edges){
        ArrayList<GraphEdge> edgelist = new ArrayList<>(edges);
        edgelist.sort((GraphEdge o1, GraphEdge o2) -> GraphUtil.compareEdgeWeights(graph, o1, o2));
        return edgelist;
    }

    /**
     * Creates a histogram, which will be used to delete a certain class, that is smaller then the given threshold.
     */
    private BinClass[] createHistogram(){
        List<GraphEdge> sortedEdges = getSortedEdges(graph, graph.edgeSet());
        double min = graph.getEdgeWeight(sortedEdges.get(sortedEdges.size() - 1));
        double max = graph.getEdgeWeight(sortedEdges.get(0));
        //getting the number of classes
        int numberOfClasses = (int) Math.ceil(1 + 3.3 * Math.log10(sortedEdges.size() - 1));
        System.out.println(numberOfClasses + " classes were build to classify the edges");
        double classRanges = (max - min) / numberOfClasses;

        BinClass[] classes = new BinClass[numberOfClasses + 1];
        for(int i = 0; i < classes.length; i++){
            classes[i] = new BinClass(sortedEdges.size());
        }

        //classify the edges
        for(GraphEdge e : sortedEdges){
            int binNumber = (int) ((graph.getEdgeWeight(e) - min) / classRanges);
            //just checks
            assert binNumber > 0 : "Edge weight is smaller then the min";
            assert binNumber < numberOfClasses : "Edge weight is bigger then the max";

            classes[binNumber].addEdge(e);
        }

        Collections.reverse(Arrays.asList(classes));

        return classes;
    }

    /**
     * Represents a class in the histogram. It holds information about the relative amount of edges it holds and the edges itself.
     */
    private class BinClass{
        private List<GraphEdge> edges;
        private int totalEdges;
        private double procent;

        BinClass(int totalEdges){
            this.edges = new ArrayList<>();
            this.totalEdges = totalEdges;
        }

        /**
         * Adds an edge to the edge list of this bucket class
         *
         * @param e - the edge to add
         */
        void addEdge(GraphEdge e){
            edges.add(e);
            procent = (edges.size() / (double) totalEdges) * 100;
        }

        /**
         * returns the number of edges this class i holding
         *
         * @return - number of edges
         */
        int nbOfEdges(){
            return edges.size();
        }

        @Override
        public String toString(){
            return edges.toString();
        }
    }

}
