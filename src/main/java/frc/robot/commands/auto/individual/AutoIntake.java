// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.individual;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoIntake extends Command {
  IntakeShooterSubsystem intake;
  SwerveSubsystem swerve;
  Timer timer;
  double speed, time, ir;

  /** Creates a new AutoIntake. */
  public AutoIntake(SwerveSubsystem swerve, IntakeShooterSubsystem intake, double speed, double time) {
    this.intake = intake;
    this.swerve = swerve;
    this.speed = speed;
    this.time = time;

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

    swerve.drive(swerve.getRobotVelocity().plus(new ChassisSpeeds(speed, 0.0, 0.0)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ir < 500;
  }
}
