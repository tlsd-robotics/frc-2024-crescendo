// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
public class IntakeSubsytem extends SubsystemBase {
  /** Creates a new IntakeSubsytem. */
  private CANSparkMax leader;
  private CANSparkMax follower;
  private ColorSensorV3 sensor; 
  public IntakeSubsytem() {
    leader = new CANSparkMax(Constants.Intake.LEADER_ID, MotorType.kBrushless);
    follower = new CANSparkMax(Constants.Intake.FOLLOWER_ID, MotorType.kBrushless);
    follower.follow(leader);
    sensor = new ColorSensorV3(I2C.Port.kOnboard);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void spin(double speed) {
    leader.set(speed); //TODO: Possible error? IIRC WPILIB Requires motors to be continuously updated for saftey.
  }
  public ColorSensorV3 getSensor() {
    return sensor;
  }
}
