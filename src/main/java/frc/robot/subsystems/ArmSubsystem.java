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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util;
import frc.robot.Constants.Arm;
import frc.robot.commands.arm.ArmToAngle;
import frc.robot.commands.arm.ArmToExtension;

public class ArmSubsystem extends SubsystemBase {
    
  /** Creates a new ArmSubsystem. */
    private CANSparkMax leader;
    private CANSparkMax follower;
    private DutyCycleEncoder encoder;
    private DoubleSolenoid armExtender;
    private DigitalInput extensionSwitch;
    private DigitalInput rotationSwitch;
    private PIDController pid;
    ArmFeedforward ff;
    private double setpoint;
    private boolean enabled = false;
    private boolean extended = false;
    private boolean targetExtension = false;
    private Timer extensionTimer = new Timer();
    private boolean profiledMotionEnabled = false;
    TrapezoidProfile armMotionProfile = new TrapezoidProfile(new Constraints(Constants.Arm.MAX_PROFILED_MOTION_VELOCITY_DEG_SEC, Constants.Arm.MAX_PROFILED_MOTION_ACCELERATION_DEG_SEC_SEC));
    TrapezoidProfile.State startState;
    TrapezoidProfile.State endState;
    Timer motionProfileTimer = new Timer();


  public ArmSubsystem() {
    enabled = false;
    leader = new CANSparkMax(Constants.Arm.LEADER_ID, MotorType.kBrushless);
    follower = new CANSparkMax(Constants.Arm.FOLLOWER_ID, MotorType.kBrushless);
    follower.follow(leader);
    leader.setIdleMode(IdleMode.kBrake);
    follower.setIdleMode(IdleMode.kBrake);
    leader.setInverted(true);
    follower.setInverted(true);
    encoder = new DutyCycleEncoder(Constants.Arm.ENCODER_ID);
    encoder.setDistancePerRotation(360);

    armExtender = new DoubleSolenoid(Constants.Arm.PNEUMATICS_MODULE_ID, PneumaticsModuleType.REVPH, Constants.Arm.FORWARD_CHANNEL, Constants.Arm.REVERSE_CHANNEL);

    extensionSwitch = new DigitalInput(Constants.Arm.EXTENSIONSWITCH);
    rotationSwitch = new DigitalInput(Constants.Arm.ROTATIONSWITCH);

    pid = new PIDController(0.01, 0.005, 0.0007);
    ff = new ArmFeedforward(0, 0.01, 0, 0);
    pid.setTolerance(3);

    SmartDashboard.putBoolean("Arm Enabled: ", false);
  }

  public double getEncoderAngle() {
    return encoder.getDistance() - Constants.Arm.ENCODER_OFFSET;
  }

  public double getAngleSetpoint() {
    return setpoint;
  }

  public void setAngle(double angle, boolean profiledMotionEnabled) {

    boolean isExtended = targetExtension;

    boolean angleTooLarge = angle > Constants.Arm.MAX_ANGLE_DEGREES;
    boolean angleTooSmall = isExtended ? (angle < Constants.Arm.MIN_ANGLE_EXTENDED_DEGREES) : (angle < Constants.Arm.MIN_ANGLE_RETRACTED_DEGREES);

    if (enabled) {
      if (profiledMotionEnabled) {
        if (!angleTooLarge && !angleTooSmall) {
          startState = new TrapezoidProfile.State(getEncoderAngle(), 0);
          endState = new TrapezoidProfile.State(angle, 0);
          this.profiledMotionEnabled = true;
          motionProfileTimer.reset();
          motionProfileTimer.start();
          setpoint = angle;
        }
        else if (angleTooSmall && (angle > setpoint)) { //Allow setpoints that move closer to a safe angle if the current setpoint is unsafe
          startState = new TrapezoidProfile.State(getEncoderAngle(), 0);
          endState = new TrapezoidProfile.State(angle, 0);
          this.profiledMotionEnabled = true;
          motionProfileTimer.reset();
          motionProfileTimer.start();
          setpoint = angle;
        }
        else if (angleTooLarge && (angle < setpoint)) {
          startState = new TrapezoidProfile.State(getEncoderAngle(), 0);
          endState = new TrapezoidProfile.State(angle, 0);
          this.profiledMotionEnabled = true;
          motionProfileTimer.reset();
          motionProfileTimer.start();
          setpoint = angle;
        }
      }
      else {
        this.profiledMotionEnabled = false;
        if (!angleTooLarge && !angleTooSmall) {
          setpoint = angle;
        }
        else if (angleTooSmall && (angle > setpoint)) { //Allow setpoints that move closer to a safe angle if the current setpoint is unsafe
          setpoint = angle;
        }
        else if (angleTooLarge && (angle < setpoint)) {
          setpoint = angle;
        }
      }
    }
  }

  public void setAngle(double angle) {
    setAngle(angle, false);
  }

  public void enableArm() {
    setpoint = getEncoderAngle(); // Prevents arm from unexpectedly snapping to a new position when it is first enabled
                                  // greatly contributing to finger safety.
    this. enabled = true;  
    profiledMotionEnabled = false;
    SmartDashboard.putBoolean("Arm Enabled: ", true);
    SmartDashboard.putString("Arm Disable Reason: ", "NA");       
  }

  public void disableArm(String reason) {
    this. enabled = false;
    SmartDashboard.putBoolean("Arm Enabled: ", false);
    SmartDashboard.putString("Arm Disable Reason: ", reason);
  }

  public void disableArm() {
    disableArm("Unknown");
  }

  public boolean getExtended() {
    return extended;
  }

  public boolean getTargetExtended() {
    return targetExtension;
  }

  public void setExtened(boolean extend) {
    if (enabled) {
      if(!extend){
        if((setpoint >= Constants.Arm.MIN_ANGLE_RETRACTED_DEGREES)){
          armExtender.set(DoubleSolenoid.Value.kReverse);
          SmartDashboard.putString("Arm Extension: ", "Extended");
        }
      }
      else {
        armExtender.set(DoubleSolenoid.Value.kForward);
        SmartDashboard.putString("Arm Extension: ", "Extended");
      }
      targetExtension = extend;
      extensionTimer.reset();
      extensionTimer.start();
    }
  }
  
  public boolean getEnabledStatus() {
    return enabled;
  }

  public boolean armAtSetpoint() {
    //return pid.atSetpoint();
    double currAngle = getEncoderAngle();
    return Util.inRange(currAngle, currAngle + 3, currAngle - 3);
  }

  @Override
  public void periodic() {

    if (!encoder.isConnected()) {
      disableArm("Encoder Disconnect");
    }

    if (enabled) {
      if (profiledMotionEnabled) {
        TrapezoidProfile.State currentState = armMotionProfile.calculate(motionProfileTimer.get(), startState, endState);
        leader.set(pid.calculate(currentState.position) + ff.calculate(Math.toRadians(currentState.position), Math.toRadians(currentState.velocity)));
        if (armAtSetpoint()) {
          profiledMotionEnabled = false;
        }
      }
      else {
        leader.set(pid.calculate(getEncoderAngle(), setpoint));
      }
    }
    else {
      leader.set(0);
    }
    if (targetExtension != extended) {
      //if (targetExtension && (extensionSwitch.get() || (extensionTimer.get() > 2))) {
      if (targetExtension && (extensionTimer.get() > 2)) {
        extended = true;
      }
      else if ((targetExtension == false) && (extensionTimer.get() > 2)) { //TODO: Add code for retraction limit switch, Add Time to Constants
        extended = false;                  //      Use NEO Encoders for encoder sanity check.
      }
    }

    SmartDashboard.putNumber("Arm Angle", getEncoderAngle());
    SmartDashboard.putBoolean("Reported Arm Extension: ", extended);
  }


  public static class Setpoint {
    public double angleDegrees;
    public boolean extended;

    public Setpoint(double angleDegrees, boolean extended) {
      this.angleDegrees = angleDegrees;
      this.extended = extended; 
    }
  } 
  

  //TODO: Use Command Event Methods to Debug: https://docs.wpilib.org/en/2022/docs/software/commandbased/command-scheduler.html
  public SequentialCommandGroup GetArmToSetpointCommand(Setpoint setpoint) {
    SequentialCommandGroup command = new SequentialCommandGroup();

    if (targetExtension != extended) 
      command.addCommands(new ArmToExtension(targetExtension, this)); //Ensure subsequent commands to not start with the arm between two extension states
                                                               //because this could result in the safeties ignoring angle commands in some cases. 
    
    if (!armAtSetpoint())
      command.addCommands(new ArmToAngle(getEncoderAngle(), this)); //For similar reasons stop the arm if it is not currently at its setpoint.
                                                   

    /*if (setpoint.angleDegrees < Constants.Arm.MIN_ANGLE_RETRACTED_DEGREES) { // If setpoint is below home position
      if (setpoint.extended && (setpoint.angleDegrees > Constants.Arm.MIN_ANGLE_EXTENDED_DEGREES)) { //Setpoint must then be extended, and not below absolute minimum angle
        if (targetExtension) { //If the arm is currently extended
          if (getEncoderAngle() > Constants.Arm.ANGULAR_CHANGE_RETRACTION_THRESHOLD_DEGREES) { //If the arm needs to travel a long distance retract the intake to avoid swinging its large weight
            command.addCommands(new ArmToExtension(false, this)); //Command the arm to retract
            command.addCommands(new ArmToAngle(Constants.Arm.MIN_ANGLE_RETRACTED_DEGREES + Constants.Arm.RETRACTED_ANGLULAR_LIMIT_SAFETY_DEGREES, this)); //Command the arm to an angle a bit above home
            command.addCommands(new ArmToExtension(true, this)); //Command the arm to extend
            command.addCommands(new ArmToAngle(setpoint.angleDegrees, this)); //Command arm to final angle
          }
          else { //the arm is traveling a short distance, and is currently extended
            command.addCommands(new ArmToAngle(setpoint.angleDegrees, this)); //Command arm to final angle
          }
        }
        else { //the arm is currently retracted //LOOK HERE
          command.addCommands(new ArmToAngle(Constants.Arm.MIN_ANGLE_RETRACTED_DEGREES + Constants.Arm.RETRACTED_ANGLULAR_LIMIT_SAFETY_DEGREES, this)); //Command the arm to an angle a bit above home (//TODO: May or may not be a desirable behavior when the angle is already below safety margin of home)
          command.addCommands(new ArmToExtension(true, this)); //Command the arm to extend
          command.addCommands(new ArmToAngle(setpoint.angleDegrees, this)); //Command arm to final angle
        }
      }
      else {
        //SETPOINT IMPOSSIBLE
        //TODO: Print Message and/or Throw runtime error.
      }
    }
    else { //Setpoint is above (or at) home position
      if (setpoint.angleDegrees < Constants.Arm.MAX_ANGLE_DEGREES) { //If angle is less than maximum possible
        if (targetExtension && (Math.abs(getEncoderAngle() - setpoint.angleDegrees) > Constants.Arm.ANGULAR_CHANGE_RETRACTION_THRESHOLD_DEGREES)) { //if arm is currently extended and angular change requires retraction  
          if (getAngleSetpoint() < Constants.Arm.MIN_ANGLE_RETRACTED_DEGREES) {
            command.addCommands(new ArmToAngle(Constants.Arm.MIN_ANGLE_RETRACTED_DEGREES + Constants.Arm.RETRACTED_ANGLULAR_LIMIT_SAFETY_DEGREES, this));
          }
          command.addCommands(new ArmToExtension(false, this)); //Command the arm to retract
        }
        command.addCommands(new ArmToAngle(setpoint.angleDegrees, this)); //Command arm to final angle
        command.addCommands(new ArmToExtension(setpoint.extended, this)); //Command the arm to final extension
      }
      else {
        //SETPOINT IMPOSSIBLE
        //TODO: Print Message and/or Throw runtime error.
      }
    }*/

    boolean needsTravelRetraction = Math.abs(getAngleSetpoint() - setpoint.angleDegrees) > Constants.Arm.ANGULAR_CHANGE_RETRACTION_THRESHOLD_DEGREES;
    boolean canRetract = getAngleSetpoint() >= Arm.MIN_ANGLE_RETRACTED_DEGREES;
    boolean setpointBelowHome = setpoint.angleDegrees < Arm.MIN_ANGLE_RETRACTED_DEGREES;

    if ((needsTravelRetraction || !setpoint.extended) && canRetract) {
      command.addCommands(new ArmToExtension(false, this));
    }
    else if ((needsTravelRetraction || !setpoint.extended) && !canRetract) {
      command.addCommands(new ArmToAngle(Constants.Arm.MIN_ANGLE_RETRACTED_DEGREES, this));
      command.addCommands(new ArmToExtension(false, this));
    }

    if (setpointBelowHome) {
      if (!targetExtension || needsTravelRetraction) {
        command.addCommands(new ArmToAngle(Constants.Arm.MIN_ANGLE_RETRACTED_DEGREES, this));
        command.addCommands(new ArmToExtension(true, this));
      }
      command.addCommands(new ArmToAngle(setpoint.angleDegrees, this)); //Command arm to final angle
    }
    else {
        command.addCommands(new ArmToAngle(setpoint.angleDegrees, this)); //Command arm to final angle
        command.addCommands(new ArmToExtension(setpoint.extended, this)); //Command the arm to final extension
    }

    return command;
  }

}
