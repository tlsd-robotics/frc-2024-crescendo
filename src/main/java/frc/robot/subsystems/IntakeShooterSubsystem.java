// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util;

public class IntakeShooterSubsystem extends SubsystemBase {
  /** Creates a new IntakeShooter. */
  
  private CANSparkMax shooter1;
  private CANSparkMax shooter2;
  private SparkPIDController shooter1PID;
  private SparkPIDController shooter2PID;

  private CANSparkMax intakeLeader;
  private CANSparkMax intakeFollower;
  private ColorSensorV3 sensor; 

  private double currentShooterSpeed = 0;

  public IntakeShooterSubsystem() {
    shooter1 = new CANSparkMax(Constants.Shooter.SHOOTER_LEADER_ID, MotorType.kBrushless);
    shooter2 = new CANSparkMax(Constants.Shooter.SHOOTER_FOLLOWER_ID, MotorType.kBrushless);
    shooter1.setInverted(false);
    shooter2.setInverted(false);
    shooter1.getEncoder().setVelocityConversionFactor(1);
    shooter2.getEncoder().setVelocityConversionFactor(1);

    shooter1PID = shooter1.getPIDController();
    shooter2PID = shooter2.getPIDController();

    shooter1PID.setP    (Constants.Shooter.SHOOTER_KP);
    shooter1PID.setI    (Constants.Shooter.SHOOTER_KI);
    shooter1PID.setD    (Constants.Shooter.SHOOTER_KD);
    shooter1PID.setIZone(Constants.Shooter.SHOOTER_KIZ);
    shooter1PID.setFF   (Constants.Shooter.SHOOTER_KFF);
    shooter2PID.setP    (Constants.Shooter.SHOOTER_KP);
    shooter2PID.setI    (Constants.Shooter.SHOOTER_KI);
    shooter2PID.setD    (Constants.Shooter.SHOOTER_KD);
    shooter2PID.setIZone(Constants.Shooter.SHOOTER_KIZ);
    shooter2PID.setFF   (Constants.Shooter.SHOOTER_KFF);
    shooter1PID.setOutputRange(Constants.Shooter.SHOOTER_PID_MIN_OUT, Constants.Shooter.SHOOTER_PID_MAX_OUT);
    shooter2PID.setOutputRange(Constants.Shooter.SHOOTER_PID_MIN_OUT, Constants.Shooter.SHOOTER_PID_MAX_OUT);

    intakeLeader = new CANSparkMax(Constants.Intake.LEADER_ID, MotorType.kBrushless);
    intakeFollower = new CANSparkMax(Constants.Intake.FOLLOWER_ID, MotorType.kBrushless);
    intakeFollower.follow(intakeLeader);
    intakeLeader.setInverted(false);
    intakeFollower.setInverted(false);

    sensor = new ColorSensorV3(I2C.Port.kOnboard);

    SmartDashboard.putNumber("Shooter Speed: ", -2);
    SmartDashboard.putNumber("Intake Speed: ",  -2);
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeSpeed(double speed) {
    intakeLeader.set(speed);
    SmartDashboard.putNumber("Intake Speed: ", speed);
  }

  public void setShooterSpeed(double speed) {
    shooter1PID.setReference(speed * Constants.Shooter.MAX_MOTOR_RPM, ControlType.kVelocity);
    shooter2PID.setReference(speed * Constants.Shooter.MAX_MOTOR_RPM, ControlType.kVelocity);
    currentShooterSpeed = speed;
    SmartDashboard.putNumber("Shooter Speed: ", speed);
  }

  public boolean shooterAtSetpoint() {
    double currentRPM = currentShooterSpeed * Constants.Shooter.MAX_MOTOR_RPM;
    return Util.inRange(shooter1.getEncoder().getVelocity(), currentRPM - Constants.Shooter.RPM_TOLERANCE, currentRPM + Constants.Shooter.RPM_TOLERANCE) &&
           Util.inRange(shooter2.getEncoder().getVelocity(), currentRPM - Constants.Shooter.RPM_TOLERANCE, currentRPM + Constants.Shooter.RPM_TOLERANCE);
  }

  public ColorSensorV3 getSensor() {
    return sensor;
  }
}
