package GMS_utils.helper;

import org.jgrapht.graph.SimpleWeightedGraph;
import org.jgrapht.graph.Subgraph;
import org.w3c.dom.Document;
import org.w3c.dom.Element;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import java.io.File;
import java.util.HashMap;
import java.util.List;

public class GraphExporter {

    /**
     * Exports a given give into a gxml file.
     *
     * @param mstGraph - the graph
     * @param path - the path to the output file
     * @param imageName - the name of the output file
     * @param graphName - the name of the graph
     * @throws ParserConfigurationException
     * @throws TransformerException
     */
    public static void export2XML(Subgraph<GraphVertex, GraphEdge, SimpleWeightedGraph<GraphVertex, GraphEdge>> mstGraph,
                                  String path,
                                  String imageName,
                                  String graphName)
            throws ParserConfigurationException, TransformerException {

        Document doc = createXMLFile();
        Element rootElement = createRootElement(doc, graphName);

        int i = 0;
        HashMap<PointHD2, Integer> nodeMap = new HashMap<>(mstGraph.vertexSet().size());
        // all the nodes
        for (PointHD2 n : mstGraph.vertexSet()) {
            rootElement.appendChild(createNode(doc, n, i));
            nodeMap.put(n, i);
            i++;
        }

        // all the edges
        for (GraphEdge e : mstGraph.edgeSet()) {
            Element edge = doc.createElement("edge");
            edge.setAttribute("source", String.valueOf(nodeMap.get(mstGraph.getEdgeSource(e))));
            edge.setAttribute("target", String.valueOf(nodeMap.get(mstGraph.getEdgeTarget(e))));
            edge.setAttribute("deleted", String.valueOf(e.isDeleted()));
            rootElement.appendChild(edge);
        }
        writeXMLFile(doc, path, imageName, graphName);
    }

    public static void exportNodeList(List<PointHD2> nodes,
                                      String path,
                                      String imageName,
                                      String graphName)
            throws ParserConfigurationException, TransformerException {
        Document doc = createXMLFile();
        Element rootElement = createRootElement(doc, graphName);

        int i = 0;
        // all the nodes
        for (PointHD2 n : nodes) {
            rootElement.appendChild(createNode(doc, n, i));
            i++;
        }
        writeXMLFile(doc, path, imageName, graphName);
    }

    private static Document createXMLFile() throws ParserConfigurationException {

        DocumentBuilderFactory docFactory = DocumentBuilderFactory.newInstance();
        DocumentBuilder docBuilder = docFactory.newDocumentBuilder();

        // root element
        Document doc = docBuilder.newDocument();
        return doc;
    }

    private static Element createRootElement(Document doc, String graphName) {

        Element rootElement = doc.createElement("graph");
        rootElement.setAttribute("class", graphName);
        doc.appendChild(rootElement);
        return rootElement;
    }

    private static void writeXMLFile(Document doc,
                                     String path,
                                     String imageName,
                                     String graphName) throws TransformerException {


        // write the content into xml file
        TransformerFactory transformerFactory = TransformerFactory.newInstance();
        Transformer transformer = transformerFactory.newTransformer();
        transformer.setOutputProperty(OutputKeys.INDENT, "yes");
        transformer.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", "4");
        DOMSource source = new DOMSource(doc);


        // if the directory does not exist, create it
        File theDir = new File(path);
        if (!theDir.exists()) {
            theDir.mkdir();
        }
        StreamResult result;
        if(path.isEmpty()){
            result = new StreamResult(new File(imageName + "_" + graphName + ".gxml"));
        } else {
            result = new StreamResult(new File(path + File.separator + imageName + "_" + graphName + ".gxml"));
        }
        // Output to console for testing
        //   StreamResult result = new StreamResult(System.out);

        transformer.transform(source, result);
    }


    //---------------

    private static Element createNode(Document doc, PointHD2 n, int i) {
        Element node = doc.createElement("node");
        node.setAttribute("id", String.valueOf(i));
        node.setAttribute("x", String.valueOf(n.getX()));
        node.setAttribute("y", String.valueOf(n.getY()));
        return node;
    }
}
