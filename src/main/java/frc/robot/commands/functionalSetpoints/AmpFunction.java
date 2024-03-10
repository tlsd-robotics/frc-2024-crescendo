// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.functionalSetpoints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Shooter.ShooterOn;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.ShooterSubsytem;

public class AmpFunction extends Command {
  ShooterSubsytem shooter;
  IntakeSubsytem intake;
  ArmSubsystem arm;
  /** Creates a new ShootFunction. */
  public AmpFunction(ShooterSubsytem shooter, IntakeSubsytem intake, ArmSubsystem arm) {
    this.shooter = shooter;
    this.intake = intake;
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SequentialCommandGroup ampFunctionCommands = new SequentialCommandGroup();
    ampFunctionCommands.addCommands(arm.GetArmToSetpointCommand(Constants.Setpoints.AMP));
    ampFunctionCommands.addCommands(new IntakeOn(intake));
    ampFunctionCommands.addCommands(new ShooterOn(intake, shooter, Constants.Intake.DEFAULT_SPEED, Constants.Shooter.DEFAULT_INTAKE_SPEED));

    CommandScheduler.getInstance().schedule(ampFunctionCommands);
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
