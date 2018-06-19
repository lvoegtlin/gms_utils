package GMS_utils;

import org.apache.commons.io.FilenameUtils;
import org.openimaj.image.FImage;
import org.openimaj.image.ImageUtilities;
import org.openimaj.image.processing.threshold.OtsuThreshold;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

/**
 * source: https://github.com/lunactic/DIVAServices_Docker_Examples/blob/master/otsubinarization/src/main/java/ch/unifr/diuf/diva/OtsuBinarization.java
 */
public class Binarization {
    /***
     * Entry point for OtsuBinarization

     */
    public static BufferedImage binarize(BufferedImage bufferedImage, String input,String output) {

        try {
            FImage inputImage = ImageUtilities.createFImage(bufferedImage);
            OtsuThreshold otsuThreshold = new OtsuThreshold();
            otsuThreshold.processImage(inputImage);
            BufferedImage outputImage = ImageUtilities.createBufferedImageForDisplay(inputImage);
            String ext = FilenameUtils.getExtension(input);
            //ByteArrayOutputStream os = new ByteArrayOutputStream();
            File outputFile = new File(output + "otsuBinaryImage." + ext);
            ImageIO.write(outputImage,ext, outputFile);
            return outputImage;
        } catch (IOException e) {
            e.printStackTrace();
        }

        return null;
    }
}
