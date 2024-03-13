// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.group;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Shooter.ShooterOn;
import frc.robot.commands.arm.ArmToExtension;
import frc.robot.commands.auto.individual.AlignWithNote;
import frc.robot.commands.auto.individual.ArmToShooterAngle;
import frc.robot.commands.auto.individual.AutoAimShooter;
import frc.robot.commands.auto.individual.DriveAutoIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class doubleNoteAuto extends SequentialCommandGroup {
  /** Creates a new singleNoteAuto. */
  public doubleNoteAuto(ArmSubsystem arm, SwerveSubsystem swerve, IntakeShooterSubsystem intakeShooter) {
    addCommands(
      arm.GetArmToSetpointCommand(Constants.Setpoints.DISENGAGE_SUPPORT),
      new AutoAimShooter(swerve, intakeShooter, 0.25),
      new ParallelCommandGroup(new ArmToShooterAngle(arm), new ArmToExtension(true, arm)),
      new ShooterOn(intakeShooter, Constants.Shooter.DEFAULT_INTAKE_SPEED, Constants.Shooter.DEFAULT_SHOOT_SPEED, Constants.Shooter.DEFAULT_DELAY),
      new AlignWithNote(swerve, 0.25, 0.8, false),
      arm.GetArmToSetpointCommand(Constants.Setpoints.INTAKE),
      new DriveAutoIntake(swerve, intakeShooter, 0.25, Constants.Shooter.DEFAULT_INTAKE_SPEED, 4),
      arm.GetArmToSetpointCommand(Constants.Setpoints.HOME),
      new AutoAimShooter(swerve, intakeShooter, 0.25),
      new ParallelCommandGroup(new ArmToShooterAngle(arm), new ArmToExtension(true, arm)),
      new ShooterOn(intakeShooter, Constants.Shooter.DEFAULT_INTAKE_SPEED, Constants.Shooter.DEFAULT_SHOOT_SPEED, Constants.Shooter.DEFAULT_DELAY)
    );
  }
}
