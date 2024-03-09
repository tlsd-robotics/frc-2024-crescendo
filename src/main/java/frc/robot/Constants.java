// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
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
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
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
  public static final double ENCODER_OFFSET = 0.0;
  public static final int LEADER_ID = 0;
  public static final int FOLLOWER_ID = 0;
  public static final DutyCycle ENCODER_ID = null;
  public static final int MODULE_ID = 0;
  public static final int FORWARD_CHANNEL = 0;
  public static final int REVERSE_CHANNEL = 0;
  public static final int EXTENSIONSWITCH = 0;
  public static final int ROTATIONSWITCH = 0;

  public static final double MAX_ANGLE_DEGREES = 180;
  public static final double MIN_ANGLE_RETRACTED_DEGREES = 15;
  public static final double MIN_ANGLE_EXTENDED_DEGREES = 0;
  public static final double MAX_MANUAL_ROTATION_RATE_DEGREES_SEC = 40;

  public static final double ANGULAR_CHANGE_RETRACTION_THRESHOLD_DEGREES = 30;
  public static final double RETRACTED_ANGLULAR_LIMIT_SAFETY_DEGREES = 5; //Amount above min retracted angle arm must extend before traveling to angles below during arm setpoint paths.
}

public static final class Setpoints { //TODO: PLACEHOLDER VALUES, TEST AND REPLACE
  public static final ArmSubsystem.Setpoint HOME    = new Setpoint(15, false);
  public static final ArmSubsystem.Setpoint INTAKE  = new Setpoint(0, true);
  public static final ArmSubsystem.Setpoint SPEAKER = new Setpoint(0, true);
  public static final ArmSubsystem.Setpoint AMP     = new Setpoint(85, true);
}

public static final class Shooter 
{
  public static final int SHOOTER_ID = 50;

  public static final double DEFAULT_SPEED = 1;
}

public static final class Intake 
{
  public static final int LEADER_ID = 51;
  public static final int FOLLOWER_ID = 52;

  public static final double DEFAULT_SPEED = 0.25;
}

}

