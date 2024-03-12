// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package TLsdLibrary.Controllers;

import edu.wpi.first.wpilibj.Joystick;

/** Add your docs here. */
public class GenericController {
    Joystick joy;
    
    public GenericController(int PortID) {
        joy = new Joystick(PortID);
    }

    public AxisSupplier getAxisSupplier(int axisId, boolean squared, double deadzone, boolean inverted) {
        return new AxisSupplier(joy, axisId, squared, deadzone, inverted);
    }
}