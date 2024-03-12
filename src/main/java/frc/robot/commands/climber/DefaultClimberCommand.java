// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class DefaultClimberCommand extends Command {
  BooleanSupplier extend;
  BooleanSupplier retract;
  ClimberSubsystem climber;
  /** Creates a new DefaultClimberCommand. */
  public DefaultClimberCommand(BooleanSupplier extend, BooleanSupplier retract, ClimberSubsystem climber) {
    this.extend = extend;
    this.retract = retract;
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (extend.getAsBoolean()) {
      climber.setExetended(true);
    }
    else if (retract.getAsBoolean()) {
      climber.setExetended(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
