package TLsdLibrary.Controllers;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;

public class AxisBooleanSupplier implements BooleanSupplier {

    boolean greaterThan;
    double triggerPoint;

    Joystick stick;
    int axisID;

    //Boolean supplier for an axis. If greaterThan is true, getAsBoolean will return true when axis value is greater than or equal to trigger point.
    //If greaterThan is false, getAsBoolen returns true when the axis is less than or equal to the trigger point
    public AxisBooleanSupplier(double triggerPoint, boolean greaterThan, Joystick stick, int axisID) {
        this.triggerPoint = triggerPoint;
        this.greaterThan = greaterThan;
        this.stick = stick;
        this.axisID = axisID;
    }

    @Override
    public boolean getAsBoolean() {
        return greaterThan ? (stick.getRawAxis(axisID) >= triggerPoint) : (stick.getRawAxis(axisID) <= triggerPoint);
    }
}