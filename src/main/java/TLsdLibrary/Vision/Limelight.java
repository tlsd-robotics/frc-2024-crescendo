package TLsdLibrary.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    int currentPipeline;
    int defaultPipeline;
    double height;
    double angle;
    
    
    
    public Limelight(int defaultPipeline, String tableName, double height, double angle) {
        this.defaultPipeline = defaultPipeline;
        this.height = height;
        this.angle = angle;
        this.table = NetworkTableInstance.getDefault().getTable(tableName);
        this.tx = table.getEntry("tx");
        this.ty = table.getEntry("ty");
        this.ta = table.getEntry("ta");
    }


    public double getHorizontalError() {
        return tx.getDouble(0.0);
    }

    public double getVerticalError() {
        return ty.getDouble(0.0);
    }

    public double getTargetArea() {
        return ta.getDouble(0.0);
    }

    public double getDistanceToTarget(Target target) {
        double angleToGoalDegrees = angle + getVerticalError();
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        return (target.getHeight() - height) / Math.tan(angleToGoalRadians);
    }

    /**
     * April tags are refered to using an ID value. 
     * @return The ID value of targeted apriltag
     */
    public double getTagID() {
        NetworkTableEntry tid = table.getEntry("tid");
        return tid.getDouble(0.0);
    }
  
    /**
     * Apriltags support 3D targeting.
     * @return Yaw value relative to robot
     */
    public double getTagYaw() {
        NetworkTableEntry camtran = table.getEntry("campose");
        double[] temp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double[] values = camtran.getDoubleArray(temp);
        return values[4];
    }
    /**
     * tv   Whether the limelight has any valid targets (0 or 1)
     * @return True if target found and vise-versa
     */
    public boolean getIsTargetFound() {
      return table.getEntry("tv").getDouble(0) > 0.5;
    }
}
