// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class IntakeOn extends Command {
  private double distance;
  private IntakeShooterSubsystem intake;
  private double speed;  
  private ColorSensorV3 sensor; 
  /** Creates a new IntakeOn. 
 * @param d 
 * @param intake */
  public IntakeOn(IntakeShooterSubsystem intake, double d) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.speed = d;
    addRequirements(intake);
  }

  public IntakeOn(IntakeShooterSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.speed = Constants.Shooter.DEFAULT_INTAKE_SPEED;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sensor = intake.getSensor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = sensor.getProximity();
    intake.setIntakeSpeed(speed);
    SmartDashboard.putNumber("Proxomity", distance);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   
    if (speed >= 0){ 
      return (distance >= Constants.Intake.NOTE_EDGE); //TODO: Move detection threshold to constants file
    } else {
      return false;
    }

  }

}
