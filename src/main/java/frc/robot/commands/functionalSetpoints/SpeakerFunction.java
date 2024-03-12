// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.functionalSetpoints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Shooter.ShooterOn;
import frc.robot.commands.Shooter.ShooterSpin;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class SpeakerFunction extends Command {
  IntakeShooterSubsystem intakeShooter;
  ArmSubsystem arm;
  /** Creates a new ShootFunction. */
  public SpeakerFunction(IntakeShooterSubsystem intakeShooter,  ArmSubsystem arm) {
    this.intakeShooter = intakeShooter;
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeShooter, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SequentialCommandGroup shootFunctionCommands = new SequentialCommandGroup();
    shootFunctionCommands.addCommands(new ShooterSpin(intakeShooter, Constants.Shooter.DEFAULT_SHOOT_SPEED)); //Spin up shooter wheels while moving arm
    shootFunctionCommands.addCommands(arm.GetArmToSetpointCommand(Constants.Setpoints.SPEAKER)); //Move arm into position
    shootFunctionCommands.addCommands(new ShooterOn(intakeShooter, Constants.Intake.DEFAULT_SPEED, Constants.Shooter.DEFAULT_SHOOT_SPEED)); //Shoot

    CommandScheduler.getInstance().schedule(shootFunctionCommands);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
