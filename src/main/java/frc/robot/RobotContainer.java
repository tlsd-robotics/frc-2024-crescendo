// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.vision.LiningUp;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.ShooterSubsytem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.arm.*;
import java.io.File;
import TLsdLibrary.Controllers.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final IntakeSubsytem intake = new IntakeSubsytem();
  private final ShooterSubsytem shooter = new ShooterSubsytem();
  private final ArmSubsystem arm = new ArmSubsystem();
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  T16000M joy = new T16000M(0);
  LogitechF310 controller = new LogitechF310(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(joy.getRawY(), OperatorConstants.Y_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(joy.getRawX(), OperatorConstants.X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(joy.getRawZ(), OperatorConstants.Z_DEADBAND),
                                                                   () -> joy.getButtonBoolean(11),
                                                                   () -> joy.getButtonBoolean(12),
                                                                   () -> joy.getButtonBoolean(13),
                                                                   () -> joy.getButtonBoolean(14));

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(joy.getRawY(), OperatorConstants.Y_DEADBAND),
        () -> MathUtil.applyDeadband(joy.getRawX(), OperatorConstants.X_DEADBAND),
        () -> joy.getRawZ(),
        () -> joy.getRawY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot f
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-joy.getRawY(), OperatorConstants.Y_DEADBAND),
        () -> MathUtil.applyDeadband(-joy.getRawX(), OperatorConstants.X_DEADBAND),
        () -> -joy.getRawZ() * 0.9);

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(joy.getRawY(), OperatorConstants.Y_DEADBAND),
        () -> MathUtil.applyDeadband(joy.getRawX(), OperatorConstants.X_DEADBAND),
        () -> joy.getRawZ() * 0.9);

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);

    arm.setDefaultCommand(new DefaultArmCommand(controller::getLeftYAxis, controller.buttonB, controller.buttonX, arm));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    joy.getTrigger().onTrue((new InstantCommand(drivebase::zeroGyro)));
    joy.getLeft().onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    // joy.getBottom().whileTrue(new LiningUp(drivebase, Vision.fronLimelight, Vision.two, joy));
    joy.getRight().onTrue((new InstantCommand(drivebase::lock)));
    joy.getBottom().whileTrue(new IntakeOn(intake, 0.5));  
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("New Path", true);
    return drivebase.getAutonomousCommand("New Path", true);
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
