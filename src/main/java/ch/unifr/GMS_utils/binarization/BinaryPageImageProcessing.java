package ch.unifr.GMS_utils.binarization;

import org.openimaj.image.FImage;
import org.openimaj.image.ImageUtilities;
import org.openimaj.image.analysis.algorithm.SummedAreaTable;
import org.openimaj.image.processing.algorithm.DifferenceOfGaussian;

import java.awt.image.BufferedImage;
import java.io.IOException;

/**
 * GraphManuscribbleTools
 * Package: ch.unifr.hisdoc2
 * Date: 07.04.16 4:41 PM
 */
public class BinaryPageImageProcessing{


    public static BufferedImage binariseImage(final BufferedImage fname,
                                              boolean useimagesettings,
                                              BinarizationAlgos binAlgo,
                                              float... binAlgoParams) throws IOException {

        BufferedImage bi = new BufferedImage(fname.getWidth(), fname.getHeight(), BufferedImage.TYPE_INT_RGB);

        switch(binAlgo){
            case SAUVOLA:
                Image img = new Image(fname);
                img.toYUV();
                Matrix m = new Matrix(img.getLayer(0));
                if(!useimagesettings)
                    binAlgoParams = new float[]{150};
                m = m.sauvolaBinarization((int) binAlgoParams[0]);
                bi = m.getImage();
                return bi;

            case OTSU:
                bi = Otsu.binarize(fname);
                if(!useimagesettings)
                    binAlgoParams = new float[]{};
                return bi;
            case DOG:
                if(!useimagesettings | binAlgoParams == null){
                    float th = 0.1f;
                    float g1 = 15f;
                    float g2 = 1.5f;

                    if(binAlgoParams.length >= 3){
                        th = binAlgoParams[0];
                    }
                    binAlgoParams = new float[]{th, g1, g2};

                }
                FImage fbi = ImageUtilities.createFImage(fname);
                FImage img2 = fbi.clone().process(new DifferenceOfGaussian(binAlgoParams[1], binAlgoParams[2]));
                img2 = img2.threshold(binAlgoParams[0]);
                return makeRGBImage(ImageUtilities.createBufferedImage(img2));
            default:
                System.out.println("Usage: java -jar PolyDrawer.jar input-image [sauvola/otsu]");
                System.exit(1);
        }

        return bi;
    }

    public static BufferedImage pageBorderRemoval(BufferedImage bi) {
        FImage image = ImageUtilities.createFImage(bi);
        float maxSearchArea = 0.2f; // percentage within to search for the page border
        SummedAreaTable integralImage = new SummedAreaTable(image);

        computeHPP(image, integralImage, 0, (int) ((image.height - 1) * maxSearchArea));
        computeHPP(image, integralImage, (int) ((image.height - 1) * (1 - maxSearchArea)), (image.height - 1));
        computeVPP(image, integralImage, 0, (int) ((image.width - 1) * maxSearchArea));
        computeVPP(image, integralImage, (int) ((image.width - 1) * (1 - maxSearchArea)), (image.width - 1));

        return makeRGBImage(ImageUtilities.createBufferedImage(image));
    }

    /**
     * compute horizontal projection profile and remove pixels
     *
     * @param image         original image, to be changed
     * @param integralImage integral image of the original image
     * @param start         start coordinate
     * @param end           end coordinate
     */
    private static void computeHPP(FImage image, SummedAreaTable integralImage, int start, int end) {
        for (int i = start; i < end; i++) {
            float sum = integralImage.calculateArea(0, i, image.width, i + 1);
            if (sum > image.height * 0.2) {
                for (int a = 0; a < image.width * 0.01; a++) {
                    for (int j = 0; j < image.width; j++) {
                        image.setPixel(j, i - a, 0f); image.setPixel(j, i + a, 0f);
                    }
                }
            }
        }
    }

    /**
     * compute vertical projection profile and remove pixels
     *
     * @param image         original image, to be changed
     * @param integralImage integral image of the original image
     * @param start         start coordinate
     * @param end           end coordinate
     */
    private static void computeVPP(FImage image, SummedAreaTable integralImage, int start, int end) {

        for (int i = start; i < end; i++) {
            float sum = integralImage.calculateArea(i, 0, i + 1, image.height);
            if (sum > image.width * 0.2) {
                for (int a = 0; a < image.height * 0.01; a++) {
                    for (int j = 0; j < image.height; j++) {
                        image.setPixel(i - a, j, 0f); image.setPixel(i + a, j, 0f);
                    }
                }
            }
        }
    }

    private static BufferedImage makeRGBImage(BufferedImage bin) {
        BufferedImage bi = new BufferedImage(bin.getWidth(), bin.getHeight(), BufferedImage.TYPE_INT_RGB);
        for (int x = 0; x < bin.getWidth(); x++) {
            for (int y = 0; y < bin.getHeight(); y++) {
                bi.setRGB(x, y, bin.getRGB(x, y));
            }
        }
        return bi;

    }
}
