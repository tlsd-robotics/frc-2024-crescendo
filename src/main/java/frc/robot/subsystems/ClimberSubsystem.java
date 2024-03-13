// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new Climber. */

  //DoubleSolenoid climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Climber.FORWARD_CHANNEL, Constants.Climber.REVERSE_CHANNEL);
  DoubleSolenoid climberSolenoid = new DoubleSolenoid(Constants.Climber.PNEUMATICS_MODULE_ID, PneumaticsModuleType.REVPH, Constants.Climber.FORWARD_CHANNEL, Constants.Climber.REVERSE_CHANNEL);

  public void setExetended(boolean extended) {
    if (extended) {
      climberSolenoid.set(Value.kForward);
      //SmartDashboard.putString("Climber State: ", "Extended");
    }
    else { 
      climberSolenoid.set(Value.kReverse);
      //SmartDashboard.putString("Climber State: ", "Retracted");
    }
  }

  public ClimberSubsystem() {
    //SmartDashboard.putString("Climber State: ", "Unknown");
  }

  @Override
  public void periodic() {}
}
