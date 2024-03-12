// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new Climber. */

  DoubleSolenoid climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Climber.FORWARD_CHANNEL, Constants.Climber.REVERSE_CHANNEL);

  public void setExetended(boolean extended) {
    if (extended)
      climberSolenoid.set(Value.kForward);
    else 
      climberSolenoid.set(Value.kReverse);
  }

  public ClimberSubsystem() {}

  @Override
  public void periodic() {}
}
