// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
ARM SUBYSTEM TODOs:
 - Startup Saftey
 -- Hold Position
 -- Prevent Setpoint Changes when
    Arm is in disabled state
*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    
  /** Creates a new ArmSubsystem. */
    private CANSparkMax leader;
    private CANSparkMax follower;
    private DutyCycleEncoder encoder;
    private DoubleSolenoid armExtender;
    private DigitalInput extensionSwitch;
    private DigitalInput rotationSwitch;
    private PIDController pid;
    private double setpoint;
    private boolean enabled;
    private boolean extended;
    private boolean targetExtensionChanged;
  public ArmSubsystem() {
    enabled = false;
    leader = new CANSparkMax(Constants.Arm.LEADER_ID, MotorType.kBrushless);
    follower = new CANSparkMax(Constants.Arm.FOLLOWER_ID, MotorType.kBrushless);
    follower.follow(leader);
    leader.setIdleMode(IdleMode.kBrake);
    follower.setIdleMode(IdleMode.kBrake);
    encoder = new DutyCycleEncoder(Constants.Arm.ENCODER_ID);
    encoder.setDistancePerRotation(360);

    armExtender = new DoubleSolenoid(Constants.Arm.MODULE_ID, PneumaticsModuleType.REVPH, Constants.Arm.FORWARD_CHANNEL, Constants.Arm.REVERSE_CHANNEL);

    extensionSwitch = new DigitalInput(Constants.Arm.EXTENSIONSWITCH);
    rotationSwitch = new DigitalInput(Constants.Arm.ROTATIONSWITCH);

    pid = new PIDController(0.01, 0, 0);
    pid.setTolerance(0.25);
    setAngle(getEncoderAngle());
  }

  public double getEncoderAngle() {
    return encoder.getDistance() - Constants.Arm.ENCODER_OFFSET;
  }

  public double getAngleSetpoint() {
    return setpoint;
  }

  public void setAngle(double angle) {

    double currentAngle = getEncoderAngle();
    boolean isExtended = getExtended();

    if(currentAngle < Constants.Arm.MAX_ANGLE_DEGREES && 
      ((isExtended ? (currentAngle > Constants.Arm.MIN_ANGLE_EXTENDED_DEGREES) : 
                     (currentAngle > Constants.Arm.MIN_ANGLE_RETRACTED_DEGREES)
       )
      )
    ) {
      setpoint = angle;
    }


  }
  public void enableArm(){
    this. enabled = true;
  }

  public void disableArm(){
    this. enabled = false;
  }

  public boolean getExtended() {
    return extended;
  }

  public void setExtened(boolean extend) {
    if (enabled) {
      if(!extend){
        if(setpoint > Constants.Arm.MIN_ANGLE_RETRACTED_DEGREES){
          armExtender.set(DoubleSolenoid.Value.kReverse);
        }
      }
      else {
        armExtender.set(DoubleSolenoid.Value.kForward);
      }
      targetExtensionChanged = true;
    }
  }
  
  public boolean getEnabledStatus() {
    return enabled;
  }

  public boolean armAtSetpoint() {
    return pid.atSetpoint();
  }

  @Override
  public void periodic() {
    if (enabled) {
      leader.set(pid.calculate(encoder.getDistance(), setpoint));
    }
    if (targetExtensionChanged) {
      if ((armExtender.get() == DoubleSolenoid.Value.kForward) && extensionSwitch.get()) {
        extended = true;
        targetExtensionChanged = false;
      }
      else if (armExtender.get() == DoubleSolenoid.Value.kReverse) {
        extended = false;
        targetExtensionChanged = false;
      }
    }

  }


  public static class Setpoint {
    double angleDegrees;
    boolean extended;

    Setpoint(double angleDegrees, boolean extended) {
      this.angleDegrees = angleDegrees;
      this.extended = extended; 
    }
  } 
  


  SequentialCommandGroup GetArmToSetpointCommand(Setpoint setpoint) {
    SequentialCommandGroup command = new SequentialCommandGroup();

    if (setpoint.angleDegrees < Constants.Arm.MIN_ANGLE_RETRACTED_DEGREES) {
      if (setpoint.extended && setpoint.angleDegrees > Constants.Arm.MIN_ANGLE_EXTENDED_DEGREES) {
        
      }
    }
    else {

    }

    return command;
  }
}
