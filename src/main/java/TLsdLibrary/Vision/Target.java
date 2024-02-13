package TLsdLibrary.Vision;

public class Target {
    double height;
    int id;
    double tolerance;
    
    public Target(double height, double tolerance, int id) {
        this.height = height;
        this.id = id;
        this.tolerance = tolerance;
    }

    public double getHeight() {
        return height;
    }

    public int getId() {
        return id;
    }

    public double getTolerance() {
        return tolerance;
    }
    
    
}
