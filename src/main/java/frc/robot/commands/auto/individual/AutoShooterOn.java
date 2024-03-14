// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.individual;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class AutoShooterOn extends Command {
  /** Creates a new ShooterOn. */
  private Timer ShootTimer = new Timer();
  private IntakeShooterSubsystem intakeShooter; 
  private double intakePower;
  private double shooterPower;
  private double value, intakeClearTime, delay; 
  private RelativeEncoder leaderEncoder, followerEncoder;
  
  
  public AutoShooterOn(IntakeShooterSubsystem intakeShooter, double intakePower, double shooterPower, double delay) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeShooter = intakeShooter; 
    this.intakePower = intakePower;
    this.shooterPower = shooterPower;
    this.delay = delay;

    addRequirements(intakeShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leaderEncoder = intakeShooter.getLeader().getEncoder();
    followerEncoder = intakeShooter.getFollower().getEncoder();

    intakeClearTime = 0.0;

    ShootTimer.reset();
    ShootTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    value = ShootTimer.get();
    if (leaderEncoder.getVelocity() >= Math.abs(shooterPower * Constants.Shooter.MAX_MOTOR_RPM) && followerEncoder.getVelocity() >= Math.abs(shooterPower * Constants.Shooter.MAX_MOTOR_RPM)) { 
      intakeShooter.setShooterSpeed(intakePower);
    }
    else {
      if (intakeClearTime == 0.0) intakeClearTime = value;
      
      intakeShooter.setIntakeSpeed(intakePower);
      intakeShooter.setShooterSpeed(shooterPower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeShooter.setShooterSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (intakeClearTime - value) >= delay;
  }
}
