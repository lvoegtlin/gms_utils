package GMS_utils.helper;

/**
 * Created by larsvoegtlin on 09.12.16.
 */
public class GraphVertex extends PointHD2{

    /**
     * If the vertex is generated or inserted by the user interaction.
     */
    private boolean userAdded = false;

    /**
     * Creates a new GraphVertex Object at {@param x} and {@param y}
     * with the polygon type {@param type}.
     *
     * @param x - double x-coordinate position
     * @param y - double y-coordinate position
     */
    public GraphVertex(float x, float y){
        super(x, y);
    }

    public GraphVertex(double x, double y){
        this((float) x, (float) y);
    }
}
