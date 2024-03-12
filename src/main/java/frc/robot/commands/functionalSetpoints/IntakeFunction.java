// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.functionalSetpoints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;


//Moves arm to intake setpoint, and performs intake cycle.

public class IntakeFunction extends Command {

  /** Creates a new IntakeFunction. */

  ArmSubsystem arm;
  IntakeShooterSubsystem intakeShooter;

  public IntakeFunction(ArmSubsystem arm, IntakeShooterSubsystem intakeShooter) {
    this.arm = arm;
    this.intakeShooter = intakeShooter;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SequentialCommandGroup intakeFunctionCommands = new SequentialCommandGroup();
    intakeFunctionCommands.addCommands(arm.GetArmToSetpointCommand(Constants.Setpoints.INTAKE));
    intakeFunctionCommands.addCommands(new IntakeOn(intakeShooter));

    CommandScheduler.getInstance().schedule(intakeFunctionCommands);
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
    return true;
  }
}
