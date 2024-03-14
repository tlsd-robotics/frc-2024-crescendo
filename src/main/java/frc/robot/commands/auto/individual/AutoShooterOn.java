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
  private IntakeShooterSubsystem intakeShooter; 
  private double intakePower;
  private double shooterPower;
  private boolean shooterAtSetpoint = false;
  private Timer shootTimer = new Timer();
  
  public AutoShooterOn(IntakeShooterSubsystem intakeShooter, double intakePower, double shooterPower) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeShooter = intakeShooter; 
    this.intakePower = intakePower;
    this.shooterPower = shooterPower;
    addRequirements(intakeShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterAtSetpoint = false;
    shootTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeShooter.setShooterSpeed(shooterPower);
    if (intakeShooter.shooterAtSetpoint()) {
      shooterAtSetpoint = true;
    }
    if (shooterAtSetpoint) {
      intakeShooter.setIntakeSpeed(intakePower);
      shootTimer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeShooter.setShooterSpeed(0.0);
    intakeShooter.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shootTimer.get() > Constants.Shooter.DEFAULT_DELAY;
  }
}