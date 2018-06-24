/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ch.unifr.GMS_utils.binarization;

import java.awt.image.BufferedImage;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

/**
 *
 * @author ms
 */
public class Matrix{
    public float[][] array;
    
    protected int width;
    protected int height;
    
    /**
     * Matrices should not be built without dimensions.
     */
    protected Matrix() {
        // nothing to do
    }
    /**
     * Creates a matrix.
     * @param width of the matrix
     * @param height of the matrix
     */
    public Matrix(int width, int height) {
        this.width = width;
        this.height = height;
        array = new float[width][height];
    }
    
    /**
     * Creates a matrix storing a copy of the given array.
     * @param arr array used as model
     */
    public Matrix(float[][] arr) {
        this(arr.length, arr[0].length);
        for (int x=0; x<width; x++) {
            System.arraycopy(arr[x], 0, array[x], 0, height);
        }
    }
    
    public int getWidth() {
        return width;
    }
    
    public int getHeight() {
        return height;
    }
    
    public BufferedImage getImage() {
        BufferedImage img = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
        
        for (int x=0; x<width; x++) {
            for (int y=0; y<height; y++) {
                if (array[x][y]>0.5) {
                    img.setRGB(x, y, 0x000000);
                } else {
                    img.setRGB(x, y, 0xFFFFFF);
                }
            }
        }
        
        return img;
    }
    
    /**
     * Returns the value of a cell.
     * @param x coordinate
     * @param y coordinate
     * @return a float
     */
    float get(int x, int y) {
        return array[x][y];
    }
    
    /**
     * Sets the value of a cell.
     * @param x coordinate
     * @param y coordinate
     * @param v new value
     */
    void set(int x, int y, float v) {
        array[x][y] = v;
    }
    
    /**
     * Creates a padded copy of the matrix.
     * @param top padding
     * @param left padding
     * @param right padding
     * @param bottom padding
     * @return a new padded matrix
     */
    public Matrix getPaddedCopy(int top, int left, int right, int bottom) {
        int w = width + left + right;
        int h = height + top + bottom;
        
        float[][] arr = new float[w][h];
        
        // copy array
        for (int x=left; x<left+width; x++) {
            for (int y=top; y<top+height; y++) {
                arr[x][y] = array[x-left][y-top];
            }
        }
        
        // extending the four corners
        for (int x=0; x<left; x++) {
            for (int y=0; y<top; y++) {
                arr[x][y] = array[0][0];
            }
            for (int y=top+height; y<h; y++) {
                arr[x][y] = array[0][height-1];
            }
        }
        for (int x=width+left; x<w; x++) {
            for (int y=0; y<top; y++) {
                arr[x][y] = array[width-1][0];
            }
            for (int y=top+height; y<h; y++) {
                arr[x][y] = array[width-1][height-1];
            }
        }
        
        // extending the borders
        for (int x=left; x<left+width; x++) {
            for (int y=0; y<top; y++) {
                arr[x][y] = arr[x][top];
            }
            for (int y=top+height; y<h; y++) {
                arr[x][y] = arr[x][top+height-1];
            }
        }
        for (int y=top; y<top+height; y++) {
            for (int x=0; x<left; x++) {
                arr[x][y] = arr[left][y];
            }
            for (int x=left+width; x<w; x++) {
                arr[x][y] = arr[left+width-1][y];
            }
        }
        
        // return a new matrix
        Matrix m = new Matrix();
        m.array = arr;
        m.width = w;
        m.height = h;
        return m;
    }
    
    /**
     * @return a copy of the matrix
     */
    public Matrix copy() {
        Matrix m = new Matrix(width, height);
        for (int x=0; x<width; x++) {
            System.arraycopy(array[x], 0, m.array[x], 0, height);
        }
        return m;
    }
    
    /**
     * Creates a copy of this matrix and applies an average filter.
     * @param window size for the filter
     * @return an averaged matrix
     */
    public Matrix averageFilter(int window) {
        // only positive odd size allowed
        if (window%2 == 0) {
            window--;
        }
        if (window<1) {
            window = 1;
        }
        
        Matrix o = copy().getPaddedCopy((window+1)/2, (window+1)/2,
                                              (window-1)/2, (window-1)/2);
        // computing the cumulative sum
        for (int x=0; x<o.width; x++) {
            for (int y=1; y<o.height; y++) {
                o.array[x][y] += o.array[x][y-1];
            }
        }
        for (int y=0; y<o.height; y++) {
            for (int x=1; x<o.width; x++) {
                o.array[x][y] += o.array[x-1][y];
            }
        }
        
        
        Matrix result = new Matrix(width, height);
        for (int x=0; x<width; x++) {
            for (int y=0; y<height; y++) {
                result.array[x][y] = (o.array[x+window][y+window]
                                   + o.array[x][y]
                                   - o.array[x+window][y]
                                   - o.array[x][y+window]) / (window*window);
            }
        }
        
        return result;
    }
    
    /**
     * @return a copy of the matrix after having applied Sauvola's text binarization
     */
    public Matrix sauvolaBinarization() {
        return sauvolaBinarization((width+height)/3);
    }
    
    /**
     * @param window size
     * @return a copy of the matrix after having applied Sauvola's text binarization with the given window size
     */
    public Matrix sauvolaBinarization(int window) {
        // threshold
        //float threshold = 0.34f;
        float threshold = 0.175f;
        
        // for now, we'll store the mean instead of the results - it avoides
        // to allocate twice the array
        Matrix result = averageFilter(window);
        
        // average of matrix.**2
        Matrix other = copy();
        for (int x=0; x<width; x++) {
            for (int y=0; y<height; y++) {
                other.array[x][y] *= other.array[x][y];
            }
        }
        other = other.averageFilter(window);
        
        // find max of standard deviation
        float max = Float.NEGATIVE_INFINITY;
        float min = Float.MAX_VALUE;
        for (int x=0; x<width; x++) {
            for (int y=0; y<height; y++) {
                other.array[x][y] = (float)sqrt(other.array[x][y] - pow(result.array[x][y],2));
                if (other.array[x][y]>max) {
                    max = other.array[x][y];
                }
                if (other.array[x][y]<min) min = other.array[x][y];
            }
        }
        
        for (int x=0; x<width; x++) {
            for (int y=0; y<height; y++) {
                // Sauvola, p. 232 : max-1, without parenthesis
                other.array[x][y] = result.array[x][y] * (1+threshold*(other.array[x][y]/max-1));
            }
        }
        
        // now we can fill the final result
        for (int x=0; x<width; x++) {
            for (int y=0; y<height; y++) {
                result.array[x][y] = (array[x][y]<other.array[x][y]) ? 0.0f : 1.0f;
            }
        }
        return result;
    }
    
    /**
     * Multiplies CELL BY CELL a matrix by another.
     * @param m other matrix
     * @throws Exception if the dimensions mismatch
     */
    public void multiplyBy(Matrix m) throws Exception {
        if (m.width!=width || m.height!=height) {
            throw new Exception("applyAnd: dimensions mismatch");
        }
        for (int x=0; x<width; x++) {
            for (int y=0; y<height; y++) {
                array[x][y] *= m.array[x][y];
            }
        }
    }
}
