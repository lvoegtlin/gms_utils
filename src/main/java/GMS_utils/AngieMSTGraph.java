package GMS_utils;

import GMS_utils.helper.*;
import boofcv.abst.feature.describe.ConfigSiftScaleSpace;
import boofcv.abst.feature.detect.interest.ConfigSiftDetector;
import boofcv.abst.feature.detect.interest.WrapSiftDetector;
import boofcv.core.image.ConvertBufferedImage;
import boofcv.factory.feature.detect.interest.FactoryInterestPoint;
import boofcv.struct.image.ImageFloat32;
import com.vividsolutions.jts.algorithm.Angle;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.triangulate.DelaunayTriangulationBuilder;
import georegression.struct.point.Point2D_F64;
import javafx.geometry.Bounds;
import javafx.scene.shape.Polygon;
import javafx.scene.shape.Rectangle;
import org.apache.commons.lang.time.StopWatch;
import org.apache.commons.math3.stat.StatUtils;
import org.jgrapht.Graph;
import org.jgrapht.alg.ConnectivityInspector;
import org.jgrapht.alg.KruskalMinimumSpanningTree;
import org.jgrapht.alg.NeighborIndex;
import org.jgrapht.graph.SimpleGraph;
import org.jgrapht.graph.SimpleWeightedGraph;
import org.jgrapht.graph.Subgraph;
import org.jgrapht.graph.UndirectedSubgraph;
import org.openimaj.image.FImage;
import org.openimaj.image.ImageUtilities;
import org.openimaj.image.connectedcomponent.ConnectedComponentLabeler;
import org.openimaj.image.contour.Contour;
import org.openimaj.image.contour.SuzukiContourProcessor;
import org.openimaj.image.feature.local.interest.HarrisIPD;
import org.openimaj.image.feature.local.interest.InterestPointData;
import org.openimaj.image.pixel.ConnectedComponent;

import java.awt.image.BufferedImage;
import java.util.*;


/**
 * class to make automatic suggestions to a user based on a graph on interest points extracted from the page
 */
public class AngieMSTGraph{

    /**
     * number of min pixels for a CC to not be discarded as noise
     */
    private int noisePx;
    /**
     * MST graph
     */
    private Subgraph<GraphVertex, GraphEdge, SimpleWeightedGraph<GraphVertex, GraphEdge>> mstGraph;
    /**
     * Stores for each point an ID indicating to which area it belongs to.
     */
    private HashMap<PointHD2, Integer> pointToArea = new HashMap<>();
    /**
     * edges that connect two different connected components.
     */
    private Set<GraphEdge> relevantEdges = new HashSet<>();

    private boolean useRelevantEdgesOnly;
    private double binsOrientationHistogram = Angle.toRadians(30);
    /**
     * selector for IP detector
     */
    private InterestPointDetector ipdSelector;
    private Distance graphDistance;
    /**
     * distributions of orientations in the document
     */
    private int[] orientationHistogram = new int[(int) (Math.PI / binsOrientationHistogram)];
    /**
     * Subgraphs of the original MST.
     */
    private List<LarsGraphCollection> subGraphs;

    private GraphCutter graphCutter;

    /**
     * create a new MST graph for the automatic suggestion of text lines
     *
     * @param noisePx number of pixels below which a component is considered noise
     */
    public AngieMSTGraph(int noisePx, boolean relevantEdgesOnly){
        this.noisePx = noisePx;
        ipdSelector = InterestPointDetector.SCP;
        graphDistance = Distance.FOCUSHORIZONTAL;
        useRelevantEdgesOnly = ipdSelector.isBinary() && relevantEdgesOnly;

        //init the subgraphslist and add the mst
        this.subGraphs = new ArrayList<>();
    }

    private void findRelevantEdges(Set<GraphEdge> edges,
                                   Graph<GraphVertex, GraphEdge> graph){
        orientationHistogram = new int[(int) (Math.PI / binsOrientationHistogram)];
        if(useRelevantEdgesOnly){
            relevantEdges.clear();
            for(GraphEdge e : edges){
                PointHD2 start = graph.getEdgeSource(e);
                PointHD2 end = graph.getEdgeTarget(e);
                if(pointToArea.get(start).equals(pointToArea.get(end))){
                    continue;
                }
                if(start.getConnectedComponentID() != end.getConnectedComponentID() || start.getConnectedComponentID() == Integer.MAX_VALUE){
                    relevantEdges.add(e);
                    int bucket = (int) ((e.getOrientation(start, end)) / binsOrientationHistogram);
                    orientationHistogram[bucket]++;
                }
            }
        } else {
            relevantEdges.clear();
            relevantEdges.addAll(edges);
        }
    }


    /**
     * triangulate the interest points extracted from a page
     *
     * @param points list of points to generate the graph from
     * @return mstGraph   mst graph to be created
     */
    private Subgraph<GraphVertex, GraphEdge, SimpleWeightedGraph<GraphVertex, GraphEdge>> createAttributedGraphsFromPage(List<PointHD2> points){
        StopWatch sw = new StopWatch();
        sw.start();
        Geometry edges = triangulatePointsJTS(points);
        sw.split();

        SimpleWeightedGraph<GraphVertex, GraphEdge> triangulatedGraph = createGraphFromTriangledEdges(edges, points);
        sw.split();
        KruskalMinimumSpanningTree<GraphVertex, GraphEdge> mst = new KruskalMinimumSpanningTree<>(triangulatedGraph);
        sw.split();

        return new Subgraph<>(triangulatedGraph, triangulatedGraph.vertexSet(), mst.getEdgeSet());
    }


    /**
     * from the triangeld edges, generate a JGraphT to process it further
     *
     * @param edges  edges of the graph
     * @param points original points
     * @return new graph
     */
    private SimpleWeightedGraph<GraphVertex, GraphEdge> createGraphFromTriangledEdges(Geometry edges, List<PointHD2> points){
        float[][] learnedDistance = getGraphDistance();

        SimpleWeightedGraph<GraphVertex, GraphEdge> graph = new SimpleWeightedGraph<>(GraphEdge.class);
        try{
            for(int i = 0; i < edges.getNumGeometries(); i++){
                Coordinate[] line = edges.getGeometryN(i).getCoordinates();
                GraphVertex v1 = new GraphVertex((float) line[0].x, (float) line[0].y);
                GraphVertex v2 = new GraphVertex((float) line[1].x, (float) line[1].y);
                graph.addVertex(v1);
                graph.addVertex(v2);
                addEdge(graph, v1, v2, learnedDistance);
            }
        } catch(Exception e1){
            e1.printStackTrace();
        }
        return graph;
    }

    /**
     * learn a distance from the points given a GT file
     *
     * @return learned matrix to shift points
     */
    private float[][] getGraphDistance(){
        // try {
        switch(graphDistance){
            case HORIZONTAL: /* Horizontal Distance */
                return Distance.HORIZONTAL.getDistance();
            case VERTICAL:   /* Vertical Distance */
                return Distance.VERTICAL.getDistance();
            case EUCLIDEAN:  /* Eucidean Distance */
                return Distance.EUCLIDEAN.getDistance();
            case FOCUSHORIZONTAL:  /* Eucidean Distance */
                return Distance.FOCUSHORIZONTAL.getDistance();
            case FOCUSVERYHORIZONTAL:  /* Eucidean Distance */
                return Distance.FOCUSVERYHORIZONTAL.getDistance();
            case MATHIAS:  /* Mathias test */
                return Distance.MATHIAS.getDistance();
            default:
                return Distance.EUCLIDEAN.getDistance();
        }
    }


    /**
     * triangulate from points
     *
     * @return a list of triangles
     */
    private Geometry triangulatePointsJTS(List<PointHD2> points){
        DelaunayTriangulationBuilder delaunay = new DelaunayTriangulationBuilder();
        delaunay.setSites(PointHD2.pointList2coordinateList(points));
        delaunay.setTolerance(0);
        return delaunay.getEdges(new GeometryFactory());
    }

    private void addEdge(SimpleWeightedGraph<GraphVertex, GraphEdge> graph, GraphVertex v1, GraphVertex v2, float[][] result){
        //TODO weight function. Make it editable
        double weight = Math.sqrt((new PointHD2(v1).matrixMultiplication(result).euclideanDistance(new PointHD2(v2).matrixMultiplication(result))));

        try{
            GraphEdge e1 = graph.addEdge(v1, v2);
            graph.setEdgeWeight(e1, weight);
        } catch(NullPointerException | IllegalArgumentException e){
            e.printStackTrace();
            // this very edge added already
            //} catch (IllegalArgumentException e) {
            // edge in the other direction added already
        }
    }


    /**
     * get points for generating the MST Tree
     *
     * @param image fimage to extract them from
     * @return list of points
     */

    private List<PointHD2> getInterestPoints(FImage image){

        switch(ipdSelector){
            case DOG:
                return doGInterestPoints(image);
            case HARRIS:
                return harrisInterestPoints(image);
           /* case DOGM: return doGInterestPoints(image, true); */
            case CP:
                return getContourPoints(image);
            case SCP:
                return getSparseContourTopologySimplifiedPoints(image);
            case CCCM:
                return getCenterOfMassCC(image);
            // case CCEP: return getFourExtremumPointsCC(image);
            default:
                throw new IllegalArgumentException("no valid interest point detector. " + ipdSelector.toString());
        }

    }

    private List<PointHD2> harrisInterestPoints(FImage image){
        float detectionscale = 2f;
        HarrisIPD harris = new HarrisIPD((float) ipdSelector.getParam(), detectionscale);
        harris.setImageBlurred(true);
        harris.findInterestPoints(image.clone());
        List<PointHD2> interestpoints = new ArrayList<>();
        while(detectionscale < 5){
            interestpoints.addAll(convertIP(harris.getInterestPoints(0.0001f)));
            detectionscale++;
            harris.setDetectionScale(detectionscale);
        }

        return interestpoints;
    }

    private List<PointHD2> convertIP(List<InterestPointData> ips){
        List<PointHD2> interestpoints = new ArrayList<>(ips.size());
        for(InterestPointData ip : ips){
            interestpoints.add(new PointHD2(ip));
        }
        return interestpoints;
    }

    private List<PointHD2> doGInterestPoints(FImage image){
        float threshold = 11f;
        WrapSiftDetector sift = (WrapSiftDetector) FactoryInterestPoint.siftDetector(new ConfigSiftScaleSpace(1.6f, 4, 1, true), // blur sigma, numscales, numoctaves, double inputimage
                new ConfigSiftDetector(2, threshold, 0, 0)); // extract radius (doesn't matter), detect th, edge th (0 = off), max features / scale (0 = off)
        BufferedImage im = ImageUtilities.createBufferedImage(image);

        sift.detect(ConvertBufferedImage.convertFrom(im, new ImageFloat32(im.getWidth(), im.getHeight())));
        List<PointHD2> interestpoints = new ArrayList<>(sift.getNumberOfFeatures());
        for(int i = 0; i < sift.getNumberOfFeatures(); i++){
            Point2D_F64 pt = sift.getLocation(i);
            interestpoints.add(new PointHD2(pt.x, pt.y));
        }

        return interestpoints;
    }


    /**
     * get contourpoints of the CC - but only sparse - i.e. not each point along the contour
     *
     * @param img image to extract them from
     * @return contour points
     */
    private List<PointHD2> getContourPoints(final FImage img){
        Contour contours = SuzukiContourProcessor.findContours(img);
        List<PointHD2> pts = new ArrayList<>(contours.size());

        int ccID = 0;
        final List<Contour> toDraw = new ArrayList<>(contours.children);

        while(!toDraw.isEmpty()){
            final Contour next = toDraw.remove(toDraw.size() - 1);

          /*  if (next.size() < noisePx / 4 || next.isHole()) {
                continue;
            }    */

            try{
                for(int i = 0; i < next.size(); i = i + 2){
                    PointHD2 p = new PointHD2(next.get(i), ccID);
                    if(p.getX() > 0 && p.getX() < img.width - 1 &&
                            p.getY() > 0 && p.getY() < img.height - 1){
                        pts.add(p);
                        pointToArea.put(p, ccID);
                    }
                }
            } catch(Exception e){
                e.printStackTrace();
            }
            //toDraw.addAll(next.children);
            ccID++;
        }
        System.out.println("Number of CC: " + ccID + ", number of nodes: " + pointToArea.size());
        return pts;
    }

    /**
     * get contourpoints of the CC - but only sparse - i.e. not each point along the contour
     *
     * @param img image to extract them from
     * @return contour points
     */
    private List<PointHD2> getSparseContourTopologySimplifiedPoints(final FImage img){
        Contour contours = SuzukiContourProcessor.findContours(img);
        List<PointHD2> pts = new ArrayList<>(contours.size());

        int ccID = 0;
        final List<Contour> toDraw = new ArrayList<>(contours.children);
        int numremoved[] = {0, 0};

        while(!toDraw.isEmpty()){
            final Contour next = toDraw.remove(toDraw.size() - 1);

            if(next.size() < noisePx / 4 || next.isHole()){
                continue;
            }
            List<PointHD2> tmpPoints = new ArrayList<>(next.size());
            try{
                for(int i = 0; i < next.size(); i += 1){
                    PointHD2 p = new PointHD2(next.get(i), ccID);
                    if(p.getX() > 0 && p.getX() < img.width - 1 &&
                            p.getY() > 0 && p.getY() < img.height - 1){
                        tmpPoints.add(p);
                    }
                }
            } catch(Exception e){
                e.printStackTrace();
            }
            try{
                tmpPoints = TopologyUtil.simplifyPointList(tmpPoints, InterestPointDetector.SCP.getParam());

            } catch(Exception ignore){
            }
            numremoved[0] += next.size();
            numremoved[1] += tmpPoints.size();
            for(PointHD2 p : tmpPoints){
                p.setccID(ccID);
                pointToArea.put(p, ccID);
            }
            pts.addAll(tmpPoints);
            //toDraw.addAll(next.children);
            ccID++;
        }
        System.out.println("Number of CC: " + ccID);
        System.out.println("Number of Nodes reduced from " + numremoved[0] + " to " + numremoved[1] + ", ratio: " + ((float) numremoved[1]) / ((float) numremoved[0]));
        return pts;
    }

    /**
     * get centroids of the CC as Interest points
     *
     * @param img image to extract them from
     * @return list of centroids
     */
    private List<PointHD2> getCenterOfMassCC(final FImage img){
        List<PointHD2> ipList = new ArrayList<>();

        ConnectedComponentLabeler ccl = new ConnectedComponentLabeler(ConnectedComponent.ConnectMode.CONNECT_8);
        List<ConnectedComponent> components = ccl.findComponents(img);

        int areaID = 0;
        for(ConnectedComponent comp : components){
            if(comp.calculateArea() < noisePx || computeMeanDistanceFromCentroid(comp) > 200){
                continue;
            }
            double[] c = comp.calculateCentroid();
            PointHD2 p = new PointHD2(c[0], c[1]);
            ipList.add(p);
            pointToArea.put(p, areaID++);
        }
        return ipList;
    }

    private double computeMeanDistanceFromCentroid(ConnectedComponent comp){
        float[] floatArray = comp.calculateBoundaryDistanceFromCentre().toArray();
        double[] doubleArray = new double[floatArray.length];
        for(int i = 0; i < floatArray.length; i++){
            doubleArray[i] = (double) floatArray[i];
        }
        return StatUtils.mean(doubleArray);
    }

    /**
     * which detector to apply
     */
    public enum InterestPointDetector{

        /*DOGM("dogMaximaOnly", 1), // use max only    */
        CP("contourPoints", 10, true),
        SCP("sparseContourPoints", 0.5, true),
        CCCM("centerOfMass", 0, true),
        /*CCEP("extremumPoints", 0)*/
        DOG("dog", 0, false),           // use max and minima
        HARRIS("harrisCorner", 3, false);

        private final String ipdName;
        private final double param;
        private final boolean binary;

        InterestPointDetector(String n, double p, boolean b){
            ipdName = n;
            param = p;
            binary = b;
        }

        double getParam(){
            return param;
        }

        public String toString(){
            return ipdName + ", " + param;
        }

        boolean isBinary(){
            return binary;
        }
    }

    /**
     * which distance to apply to the graph
     */
    public static enum Distance{
        HORIZONTAL("horizontal", new float[][]{{0, 0}, {0, 1}}),
        VERTICAL("vertical", new float[][]{{1, 0}, {0, 0}}),
        EUCLIDEAN("euclidean", new float[][]{{1, 0}, {0, 1}}),
        FOCUSHORIZONTAL("focushorizontal", new float[][]{{0.5f, 0}, {0, 1.5f}}),
        FOCUSVERYHORIZONTAL("focushorizontal", new float[][]{{0.2f, 0}, {0, 1.8f}}),
        MATHIAS("mathias", new float[][]{{0.6f, 0}, {0, 1}}),
        USERANGLE("user", new float[][]{{0, 0}, {0, 0}});
        //LEARN_NEWTON("learned");

        private final String name;

        private float[][] distance;

        Distance(String n, float[][] d){
            name = n;
            distance = d;
        }

        public String toString(){
            return name + Arrays.deepToString(distance);
        }

        public float[][] getDistance(){
            return distance;
        }

        public void setUserAngle(float angle){
            float weightx = 0.05f;
            float weighty = 0.95f;
            USERANGLE.distance = new float[][]{{(float) Math.cos(angle) * weightx, (float) -Math.sin(angle) * weightx},
                    {(float) Math.sin(angle) * weighty, (float) Math.cos(angle) * weighty}};

            System.out.println("orientation: \n" + USERANGLE.distance[0][0] + ", " + USERANGLE.distance[0][1] + "\n" + USERANGLE.distance[1][0] + ", " + USERANGLE.distance[1][1]);
        }
    }

    //////////////////////////////////////////////////////
    /* !!!!!!!!!!!!!!!!!!!! NEW CODE!!!!!!!!!!!!!!!!!!!!*/
    //////////////////////////////////////////////////////



    /**
     * Adds a subgraph to the list of subgraphs and also to the quad tree if the user wants this.
     *
     * @param graph - the graph to add
     * @param toQuadTree - true if you also want to add the graph to the quad tree
     */
    private void addNewSubgraph(LarsGraphCollection graph, boolean toQuadTree){
        subGraphs.add(graph);
    }

    /**
     * Creates the Graph based of the original and the binary picture.
     *
     * @param bimg - the binary picture
     * @param img  - the original pricture
     */
    public void createGraph(BufferedImage bimg, BufferedImage img, double percentage){
        FImage image = ipdSelector.isBinary() ? ImageUtilities.createFImage(bimg) : ImageUtilities.createFImage(img);

        List<PointHD2> points = getInterestPoints(image);

        mstGraph = createAttributedGraphsFromPage(points);
        findRelevantEdges(mstGraph.edgeSet(), mstGraph);

        this.graphCutter = new GraphCutter(mstGraph);

        forceForest(percentage);

        System.out.println("Amount of graphs: " + subGraphs.size());
    }

    /**
     * Cuts edges on the given constant threshold to create a initial forest out of the original mst.
     * It also starts all the threads to create the graphs and the concave hulls.
     */
    private void forceForest(double percentage){
        //create a copy
        Subgraph<GraphVertex, GraphEdge, SimpleWeightedGraph<GraphVertex, GraphEdge>> clone =
                new Subgraph<>(mstGraph.getBase(), mstGraph.vertexSet(), mstGraph.edgeSet());

        //cuts the edges in the original (labels) and in the clone
        graphCutter.cutHighCostEdges(clone, percentage);
        //create the undirected graph to use the connectivity inspector
        UndirectedSubgraph<GraphVertex, GraphEdge> undirectedClone = new UndirectedSubgraph<>(clone.getBase(), clone.vertexSet(), clone.edgeSet());
        //get all graphs
        ConnectivityInspector<GraphVertex, GraphEdge> cI = new ConnectivityInspector<>(undirectedClone);
        List<Set<GraphVertex>> forest = cI.connectedSets();
        //create graphs
        int biggestSubtree = 0;
        for(Set<GraphVertex> graphVertices : forest){
            if(graphVertices.size() > biggestSubtree){
                biggestSubtree = graphVertices.size();
            }
            UndirectedSubgraph<GraphVertex, GraphEdge> newGraph = new UndirectedSubgraph<>(clone.getBase(),
                    graphVertices, new HashSet<GraphEdge>());
            for(GraphVertex v : graphVertices){
                for(GraphEdge e : undirectedClone.edgesOf(v)){
                    newGraph.addEdge(undirectedClone.getEdgeSource(e), undirectedClone.getEdgeTarget(e), e);
                }
            }
            //creates new LarsGraphCollection and starts the concave hull service
            LarsGraphCollection newLarsGraphCollection = new LarsGraphCollection(new LarsGraph(newGraph));
            addNewSubgraph(newLarsGraphCollection, false);

            //TODO if we also want to print out the hulls of the graphs
            /*ConcaveHullExtractionService cHES = new ConcaveHullExtractionService();
            cHES.setOnFailed(event ->
                cHES.getException().printStackTrace(System.err)
            );
            cHES.setLarsGraphCollection(newLarsGraphCollection);
            cHES.start();*/
        }

        System.out.println("biggest subtree has " + biggestSubtree + " vertices");
    }

    /**
     * Returns the original graph where the edges have a special information about deletion.
     *
     * @return - The graph
     */
    public Subgraph<GraphVertex, GraphEdge, SimpleWeightedGraph<GraphVertex, GraphEdge>> getMstGraph(){
        return mstGraph;
    }
}
