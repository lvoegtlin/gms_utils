package ch.unifr.GMS_utils;

import ch.unifr.GMS_utils.binarization.BinarizationAlgos;
import ch.unifr.GMS_utils.binarization.BinaryPageImageProcessing;
import ch.unifr.GMS_utils.graph.AngieMSTGraph;
import ch.unifr.GMS_utils.graph.GraphExporter;
import org.apache.commons.io.FilenameUtils;

import javax.imageio.ImageIO;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.TransformerException;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

public class Gms_utils{
    /**
     * @param args 0: Path to input image
     *             1: Path to output folder
     *             2: Binarization algorithm(optional) (dog, otsu, sauvola)
     */
    public static void main(String[] args) throws IOException, TransformerException, ParserConfigurationException{
        BinarizationAlgos algo;
        if(args.length < 2){
            System.err.println("Both parameter (path to image and path to the output folder) are required!");
            System.exit(1);
        }

        try{
            algo = BinarizationAlgos.valueOf(args[2]);
        } catch(IllegalArgumentException | IndexOutOfBoundsException e){
            algo = BinarizationAlgos.DOG;
        }


        String inputPath = args[0];
        String outputPath = args[1];
        String name_raw = inputPath.substring(inputPath.lastIndexOf("/") + 1);
        String name = name_raw.substring(0, name_raw.lastIndexOf("."));

        File inputFile = new File(inputPath);

        BufferedImage ori = ImageIO.read(inputFile);
        BufferedImage bin = BinaryPageImageProcessing.binariseImage(ori, false, algo);
        ImageIO.write(bin, FilenameUtils.getExtension(name_raw),
                new File(FilenameUtils.getPath(inputPath) + name + "_binary." + FilenameUtils.getExtension(inputPath))
        );

        AngieMSTGraph graph = new AngieMSTGraph(30, true);
        graph.createGraph(bin, ori, 10);
        GraphExporter.export2XML(graph.getMstGraph(), outputPath, name + "_graph", name);
    }
}
