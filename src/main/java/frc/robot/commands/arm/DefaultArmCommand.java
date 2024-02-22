// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import TLsdLibrary.Controllers.LogitechF310;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class DefaultArmCommand extends Command {
  /** Creates a new DefaultArmCommand. */
  ArmSubsystem arm;
  LogitechF310 joy;
  PIDController pid;
  public DefaultArmCommand(ArmSubsystem arm, LogitechF310 joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.joy = joy;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

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
