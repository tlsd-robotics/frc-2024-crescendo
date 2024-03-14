// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package TLsdLibrary.Controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class GenericController {
    Joystick joy;
    
    public GenericController(int PortID) {
        joy = new Joystick(PortID);
    }

    public AxisSupplier getAxisSupplier(int axisId, boolean squared, double deadzone, boolean inverted) {
        return new AxisSupplier(joy, axisId, squared, deadzone, inverted);
    }

    public Trigger getAxisTrigger (int axisID, double triggerPoint, boolean greaterThan) {
        return new Trigger(new AxisBooleanSupplier(triggerPoint, greaterThan, joy, axisID));
    }
}
