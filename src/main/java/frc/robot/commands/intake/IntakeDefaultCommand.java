// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class IntakeDefaultCommand extends Command {

  DoubleSupplier axis;
  IntakeShooterSubsystem intakeShooter;


  /** Creates a new ShooterIntakeDefaultCommand. */
  public IntakeDefaultCommand(DoubleSupplier axis, IntakeShooterSubsystem intakeShooter) {
    this.axis = axis;
    this.intakeShooter = intakeShooter;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeShooter.setIntakeSpeed(axis.getAsDouble());
    intakeShooter.setShooterSpeed((1.0/10.0) * axis.getAsDouble());
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
