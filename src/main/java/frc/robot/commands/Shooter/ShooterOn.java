// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.ShooterSubsytem; 

public class ShooterOn extends Command {
  /** Creates a new ShooterOn. */
  private Timer ShootTimer = new Timer();
  private IntakeSubsytem intake; 
  private ShooterSubsytem shooter;
  private double intakePower;
  private double shooterPower;
  private double value; 
  
  
  public ShooterOn(IntakeSubsytem intake, ShooterSubsytem shooter, double intakePower, double shooterPower) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake; 
    this.shooter = shooter; 
    this.intakePower = intakePower;
    this.shooterPower = shooterPower;
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShootTimer.reset();
    ShootTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    value = ShootTimer.get();
    if (value < .5) { //TODO: Transition to rpm based PID control rather than inconsistent timer based system
      shooter.spin(shooterPower);
    }
    else {
      intake.spin(intakePower);
      shooter.spin(shooterPower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.spin(0.0);
    shooter.spin(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //TODO: Detect when completed
  }
}
