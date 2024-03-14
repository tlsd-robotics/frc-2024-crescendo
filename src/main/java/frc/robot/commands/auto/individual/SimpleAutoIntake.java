// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.individual;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SimpleAutoIntake extends Command {
  IntakeShooterSubsystem intake;
  SwerveSubsystem swerve;
  Timer timer;
  double speed, time, ir;

  /** Creates a new AutoIntake. */
  public SimpleAutoIntake(SwerveSubsystem swerve, IntakeShooterSubsystem intake, double speed, double abandonTime) {
    this.intake = intake;
    this.swerve = swerve;
    this.speed = speed;
    this.time = abandonTime;

    addRequirements(swerve, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ir = intake.getSensor().getProximity();

    intake.setIntakeSpeed(Constants.Shooter.DEFAULT_INTAKE_SPEED);
    swerve.drive(new ChassisSpeeds(speed, 0.0, 0.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    intake.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (ir < Constants.Intake.NOTE_EDGE) || (timer.get() > time);
  }
}
