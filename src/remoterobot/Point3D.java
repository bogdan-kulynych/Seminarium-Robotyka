/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package remoterobot;

/**
 *
 * @author stefan
 */
class Point3D {
    
    double X, Y, Z;
    public Point3D(double x, double y, double z) {
        X = x; Y = y; Z = z;
    }

    public double getX() {
        return X;
    }

    public double getY() {
        return Y;
    }

    public double getZ() {
        return Z;
    }
        
}
