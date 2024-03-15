// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Setpoints;
import frc.robot.commands.HaltArmShooterIntake;
import frc.robot.commands.Shooter.ShooterOn;
import frc.robot.commands.Shooter.ShooterSpin;
import frc.robot.commands.intake.IntakeDefaultCommand;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.commands.intake.IntakeShooterSpin;
import frc.robot.commands.vision.AimIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.arm.*;
import frc.robot.commands.auto.individual.AutoShooterOn;
import frc.robot.commands.auto.individual.SimpleAutoIntake;
import frc.robot.commands.climber.ClimberSet;
import frc.robot.commands.climber.DefaultClimberCommand;
import frc.robot.commands.functionalSetpoints.HomeFunction;
import frc.robot.commands.functionalSetpoints.SpeakerFunction;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import TLsdLibrary.Controllers.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */

 //The default positive direction for a neo is clockwise when viewed from the back of the motor.
 
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final IntakeShooterSubsystem intakeShooter = new IntakeShooterSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  T16000M joy = new T16000M(0);
  LogitechF310 controller = new LogitechF310(1);


  SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    configureAutoCommands();

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

    arm.setDefaultCommand(new DefaultArmCommand(controller.getAxisSupplier(controller.leftYAxis, false, 0.02, true), controller.getDPadRight(), controller.getDPadLeft(), arm));
    //arm.setDefaultCommand(new DefaultArmCommand(controller::getLeftYAxis, controller.buttonB, controller.buttonX, arm));
    intakeShooter.setDefaultCommand(new IntakeDefaultCommand(controller.getAxisSupplier(controller.rightXAxis, false, 0.02, true), intakeShooter));
    climber.setDefaultCommand(new DefaultClimberCommand(controller.dPadUp, controller.dPadDown, climber));

    //Creates a sendable chooser using all autos paths in roborio delploy folder. See:
    // https://pathplanner.dev/pplib-build-an-auto.html#create-a-sendablechooser-with-all-autos-in-project
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous Chooser", autoChooser);

    CommandScheduler.getInstance().onCommandInitialize(command -> System.out.println("Command Initalized: " + command.getName()));
    CommandScheduler.getInstance().onCommandInterrupt(command -> System.out.println("Command Interrupted: " + command.getName()));
    CommandScheduler.getInstance().onCommandFinish(command -> System.out.println("Command Finished: " + command.getName()));

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
    //joy.getLeft().onTrue(new InstantCommand(drivebase::addFakeVisionReading));

    joy.getTrigger().whileTrue(new ShooterOn(intakeShooter, Constants.Shooter.DEFAULT_INTAKE_SPEED, Constants.Shooter.DEFAULT_SHOOT_SPEED));
    //joy.getBottom().whileTrue(new ParallelCommandGroup(new AimIntake(drivebase, joy, 3), new IntakeOn(intakeShooter)));
    joy.getBottom().whileTrue(new IntakeOn(intakeShooter));
    joy.getRight().onTrue((new InstantCommand(drivebase::lock, drivebase)));
    joy.getLeft().onTrue((new InstantCommand(drivebase::zeroGyro)));
    joy.POVUp.onTrue(new ClimberSet(true, climber));
    joy.POVDown.onTrue(new ClimberSet(false, climber));

    //controller.buttonA.onTrue(new HomeFunction(arm));
    //controller.buttonA.onFalse(new HaltArmShooterIntake(intakeShooter, arm));
    //controller.buttonB.whileTrue(new ShooterSpin(intakeShooter, 1));

    controller.buttonA.whileTrue(arm.GetArmToSetpointCommand(Constants.Setpoints.HOME));
    controller.buttonB.whileTrue(arm.GetArmToSetpointCommand(Constants.Setpoints.INTAKE));
    controller.buttonX.whileTrue(arm.GetArmToSetpointCommand(Constants.Setpoints.SPEAKER));
    controller.buttonY.whileTrue(arm.GetArmToSetpointCommand(Constants.Setpoints.AMP));
    
    controller.getAxisTrigger(controller.rightTrigger, 0.9, true).whileTrue(new IntakeShooterSpin(intakeShooter, Constants.Shooter.DEFAULT_INTAKE_SPEED, Constants.Shooter.DEFAULT_INTAKE_SPEED * Constants.Shooter.INTAKE_RELATIVE_SPEED_RATIO));
    controller.getAxisTrigger(controller.leftTrigger, 0.9, true).whileTrue(new ShooterOn(intakeShooter, 0, 0));
    controller.buttonLB.whileTrue(new SequentialCommandGroup ( //Allows override of PID wait
        new IntakeShooterSpin(intakeShooter, Constants.Shooter.DEFAULT_SHOOT_SPEED, 0),
        new WaitCommand(1),
        new IntakeShooterSpin(intakeShooter, Constants.Shooter.DEFAULT_SHOOT_SPEED, Constants.Shooter.DEFAULT_SHOOT_SPEED)
      )
    );
    controller.buttonRB.whileTrue(new IntakeOn(intakeShooter));

    controller.buttonBack.onTrue(new InstantCommand(arm::disableArm));
    controller.buttonStart.onTrue(new InstantCommand(arm::enableArm));
  }

  private void configureAutoCommands() {
    //Register commands for use in autonomous as follows:
    //NamedCommands.registerCommand("Name in Pathplanner", command);
    NamedCommands.registerCommand("SimpleAutoIntake", new SimpleAutoIntake(drivebase, intakeShooter, 0.1, 10));
    NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(arm.GetArmToSetpointCommand(Setpoints.SPEAKER), new AutoShooterOn(intakeShooter, Constants.Shooter.DEFAULT_SHOOT_SPEED, Constants.Shooter.DEFAULT_SHOOT_SPEED)));
    NamedCommands.registerCommand("ArmRetract", arm.GetArmToSetpointCommand(Setpoints.HOME));
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
    //return drivebase.getAutonomousCommand("New Path", true);
    return new SequentialCommandGroup(arm.GetArmToSetpointCommand(Constants.Setpoints.DISENGAGE_SUPPORT), autoChooser.getSelected());
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  //called when robot enters teleop
  public void onTeleop() {
    arm.enableArm();
  }

  //called when robot enters autonomous
  public void onAutonomous() {
    arm.enableArm();
  }

  //called when robot is disabled
  public void onDisable() {
    arm.disableArm();
  }
}
