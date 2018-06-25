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
import java.util.ArrayList;
import java.util.Arrays;

public class Gms_utils{
    /**
     * @param args 0: Path to input folder
     *             1: Binarization algorithm(optional) (dog, otsu, sauvola)
     */
    public static void main(String[] args) throws IOException, TransformerException, ParserConfigurationException{
        BinarizationAlgos algo;
        boolean info = false;
        if(args.length < 1){
            System.err.println("The parameter (path to input folder) is required!");
            System.exit(1);
        }

        try{
            algo = BinarizationAlgos.valueOf(args[1]);
        } catch(IllegalArgumentException | IndexOutOfBoundsException e){
            algo = BinarizationAlgos.DOG;
        }

        ArrayList<String> extensions = new ArrayList<>(Arrays.asList("jpg", "JPG", "PNG", "png"));

        String inputPath = args[0];

        File inputDirectory = new File(inputPath);
        if(!inputDirectory.exists()){
            System.err.println("Given directory is not existing!");
            System.exit(1);
        }

        for(File f : inputDirectory.listFiles()){
            if(!f.isDirectory()){
                if(extensions.contains(FilenameUtils.getExtension(f.getName()))){
                    File dir = new File(f.getParent() + "/"+FilenameUtils.getBaseName(f.getName()));
                    System.out.println("Creating directory...");
                    dir.mkdir();
                    export(f, dir,  algo);
                } else {
                    info = true;
                }
            }
            System.out.println("---------------");
        }

        if(info){
            System.out.println("Allowed file extensions: ");
            for(String s : extensions){
                System.out.println(s);
            }
        }

        System.out.println("Finished");
    }

    private static void export(File inputFile, File parent, BinarizationAlgos algo) throws IOException, ParserConfigurationException, TransformerException{
        String baseName = FilenameUtils.getBaseName(inputFile.getName());
        String parentString = parent.getPath();
        BufferedImage ori = ImageIO.read(inputFile);
        ImageIO.write(ori, FilenameUtils.getExtension(inputFile.getName()),
                new File(parentString + "/" + baseName + "." + FilenameUtils.getExtension(inputFile.getName()))
        );
        System.out.println("Start binarization...");
        BufferedImage bin = BinaryPageImageProcessing.binariseImage(ori, false, algo);
        System.out.println("Save binarized image...");
        ImageIO.write(bin, FilenameUtils.getExtension(inputFile.getName()),
                new File(parentString + "/" + baseName + "_binary." + FilenameUtils.getExtension(inputFile.getName()))
        );

        System.out.println("Start graph creation...");
        AngieMSTGraph graph = new AngieMSTGraph(30, true);
        System.out.println("---Graph creation---");
        graph.createGraph(bin, ori, 10);
        System.out.println("---Graph finished---");
        System.out.println("Save graph...");
        GraphExporter.export2XML(graph.getMstGraph(), parentString + "/", baseName + "_graph", baseName);
    }
}
