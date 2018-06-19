package GMS_utils.helper;

import org.jgrapht.graph.SimpleWeightedGraph;
import org.jgrapht.graph.Subgraph;

/**
 * several Utility methods for graphs
 */
class GraphUtil{

    /**
     * comparator for two edges
     *
     * @param graph graph the edges are from
     * @param o1    edge 1
     * @param o2    edge 2
     * @return -1 if o1 > o2, 0 if o1==02, 1 otherwise
     */
    public static <N extends PointHD2, E extends GraphEdge, SG extends Subgraph<N,E,SimpleWeightedGraph<N, E>>> int compareEdgeWeights(final SG graph, E o1, E o2) {
        return Double.compare(graph.getEdgeWeight(o2), graph.getEdgeWeight(o1));
    }
}
