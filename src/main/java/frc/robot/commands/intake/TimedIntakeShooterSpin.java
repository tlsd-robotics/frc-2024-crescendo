// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class TimedIntakeShooterSpin extends Command {

  double shooterSpeed;
  double intakeSpeed;
  IntakeShooterSubsystem intakeShooter;
  double timeSeconds;
  Timer timer = new Timer();


  /** Creates a new ShooterIntakeDefaultCommand. */
  public TimedIntakeShooterSpin(IntakeShooterSubsystem intakeShooter, double intakeSpeed, double shooterSpeed, double timeSeconds) {
    this.intakeSpeed = intakeSpeed;
    this.shooterSpeed = shooterSpeed;
    this.intakeShooter = intakeShooter;
    this.timeSeconds = timeSeconds;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeShooter.setIntakeSpeed(intakeSpeed);
    intakeShooter.setShooterSpeed(shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= timeSeconds;
  }
}
