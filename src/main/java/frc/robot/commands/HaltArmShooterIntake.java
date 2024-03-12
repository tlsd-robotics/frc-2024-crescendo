// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class HaltArmShooterIntake extends Command {
  /** Creates a new HaltArmShooterIntake. */

  ArmSubsystem arm;
  IntakeShooterSubsystem intakeShooter;

  //Will interrupt any executing arm/intake/shooter commands which are interruptable.
  //Stops Intake and Shooter wheels
  public HaltArmShooterIntake(IntakeShooterSubsystem intakeShooter) {
    this.intakeShooter = intakeShooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, intakeShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeShooter.setIntakeSpeed(0);  //Likley Redundant in most cases
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
