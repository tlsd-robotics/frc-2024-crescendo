// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeShooterSubsystem extends SubsystemBase {
  /** Creates a new IntakeShooter. */
  
  private CANSparkMax shooterLeader;
  private CANSparkMax shooterFollower;

  private CANSparkMax intakeLeader;
  private CANSparkMax intakeFollower;
  private ColorSensorV3 sensor; 

  public IntakeShooterSubsystem() {
    shooterLeader = new CANSparkMax(Constants.Shooter.SHOOTER_LEADER_ID, MotorType.kBrushless);
    shooterFollower = new CANSparkMax(Constants.Shooter.SHOOTER_FOLLOWER_ID, MotorType.kBrushless);
    shooterLeader.setInverted(false);
    shooterFollower.setInverted(false);

    intakeLeader = new CANSparkMax(Constants.Intake.LEADER_ID, MotorType.kBrushless);
    intakeFollower = new CANSparkMax(Constants.Intake.FOLLOWER_ID, MotorType.kBrushless);
    intakeFollower.follow(intakeLeader);
    intakeLeader.setInverted(false);
    intakeFollower.setInverted(false);

    sensor = new ColorSensorV3(I2C.Port.kOnboard);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeSpeed(double speed) {
    intakeLeader.set(speed);
  }

  public void setShooterSpeed(double speed) {
    shooterLeader.set(speed);
  }

  public ColorSensorV3 getSensor() {
    return sensor;
  }

  public CANSparkMax getLeader() {
    return shooterLeader;
  }

    public CANSparkMax getFollower() {
    return shooterFollower;
  }
}
