// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Setpoint;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(Units.inchesToMeters(25), Units.inchesToMeters(20), Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double X_DEADBAND  = 0.01;
    public static final double Y_DEADBAND  = 0.01;
    public static final double Z_DEADBAND = 0.01;
    public static final double TURN_CONSTANT    = 6;
  }
  
public static final class Arm
{
  public static final double ENCODER_OFFSET = 41.8;
  public static final int LEADER_ID = 25;
  public static final int FOLLOWER_ID = 22;
  public static final int ENCODER_ID = 0;
  public static final int PNEUMATICS_MODULE_ID = 2;
  public static final int FORWARD_CHANNEL = 3;
  public static final int REVERSE_CHANNEL = 2;
  public static final int EXTENSIONSWITCH = 1;
  public static final int ROTATIONSWITCH = 2;

  public static final double MAX_ANGLE_DEGREES = 95;
  public static final double MIN_ANGLE_RETRACTED_DEGREES = 21;
  public static final double MIN_ANGLE_EXTENDED_DEGREES = 0;
  public static final double MAX_MANUAL_ROTATION_RATE_DEGREES_SEC = 40;

  public static final double ANGULAR_CHANGE_RETRACTION_THRESHOLD_DEGREES = 30;
  public static final double RETRACTED_ANGLULAR_LIMIT_SAFETY_DEGREES = 5; //Amount above min retracted angle arm must extend before traveling to angles below during arm setpoint paths.
}

public static final class Setpoints { //TODO: PLACEHOLDER VALUES, TEST AND REPLACE
  public static final ArmSubsystem.Setpoint HOME                 = new Setpoint(21, false);
  public static final ArmSubsystem.Setpoint INTAKE               = new Setpoint(0, true);
  public static final ArmSubsystem.Setpoint SPEAKER              = new Setpoint(0, true);
  public static final ArmSubsystem.Setpoint AMP                  = new Setpoint(95, true);
  public static final ArmSubsystem.Setpoint DISENGAGE_SUPPORT    = new Setpoint(75, false);
}

public static final class Shooter 
{
  public static final int SHOOTER_LEADER_ID = 24;
  public static final int SHOOTER_FOLLOWER_ID = 17;

  public static final double WHEEL_DIAMETER_INCHES = 4;
  public static final double DEFAULT_SHOOT_SPEED = 0.7;
  public static final double DEFAULT_INTAKE_SHOOT_SPEED = 1;
  public static final double DEFAULT_INTAKE_SPEED = 0.6;
  public static final double DEFAULT_DELAY = 2;

  public static final double MAX_MOTOR_RPM = 5676;

  public static final double MAX_RANGE = 2;

  public static final double SHOOTER_KP  = 5e-4;
  public static final double SHOOTER_KI  = 0.000001;
  public static final double SHOOTER_KD  = 0; 
  public static final double SHOOTER_KIZ = 0;
  public static final double SHOOTER_KFF = 0.000015;

  public static final double SHOOTER_PID_MAX_OUT = 1;
  public static final double SHOOTER_PID_MIN_OUT = -1;

  public static final double RPM_TOLERANCE = 10;

  public static final double INTAKE_RELATIVE_SPEED_RATIO = (1.0/10.0);
  
}

public static final class Intake 
{
  public static final int LEADER_ID = 16;
  public static final int FOLLOWER_ID = 23;

  public static final double WHEEL_DIAMETER_INCHES = 2;
  public static final double DEFAULT_SPEED = 0.25;

  public static final double NOTE_EDGE = 110;
  public static final double NOTE_CENTER = 500;
}

public static final class Climber 
{
  public static final int PNEUMATICS_MODULE_ID = 2;
  public static final int FORWARD_CHANNEL = 1;
  public static final int REVERSE_CHANNEL = 0;
}
public static final class Superstructure
{
  public static final int PH_CAN_ID = 2;
  public static final int PDP_CAN_ID = 1;
}

public static final class Vision 
{
  public static final Translation3d ROBOT_TO_CAM_TRANSLATION = new Translation3d(0.5, 0.0, 0.0); // X = forward, Y = left, Z = up
  public static final Rotation3d ROBOT_TO_CAM_ROTATION = new Rotation3d(0.0, 0.0, 0.0);

  public static final double INTAKE_SETPOINT = -20.0;
}
}

