// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class DefaultArmCommand extends Command {
  /** Creates a new DefaultArmCommand. */

  DoubleSupplier rotationInput;
  BooleanSupplier extend;
  BooleanSupplier retract;

  ArmSubsystem arm;

  Timer timer = new Timer();

  public DefaultArmCommand(DoubleSupplier rotationInput, BooleanSupplier extend, BooleanSupplier retract,  ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.rotationInput = rotationInput;
    this.extend = extend;
    this.retract = retract;
    this.arm = arm;
    
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    arm.setAngle(arm.getAngleSetpoint() + rotationInput.getAsDouble() * Constants.Arm.MAX_MANUAL_ROTATION_RATE_DEGREES_SEC * timer.get());
    SmartDashboard.putNumber("Arm Setpoint", arm.getAngleSetpoint());
    SmartDashboard.putNumber("Arm Rotation Input", rotationInput.getAsDouble());
    timer.reset();

    if(extend.getAsBoolean()) {
        arm.setExtened(true);
    }
    else if(retract.getAsBoolean()) {
        arm.setExtened(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
