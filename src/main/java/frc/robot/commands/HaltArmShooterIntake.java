// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.ShooterSubsytem;

public class HaltArmShooterIntake extends Command {
  /** Creates a new HaltArmShooterIntake. */

  ArmSubsystem arm;
  IntakeSubsytem intake;
  ShooterSubsytem shooter;

  //Will interrupt any executing arm/intake/shooter commands which are interruptable.
  //Stops Intake and Shooter wheels
  public HaltArmShooterIntake(ArmSubsystem arm, IntakeSubsytem intake, ShooterSubsytem Shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.spin(0);  //Likley Redundant in most cases
    shooter.spin(0); //
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
