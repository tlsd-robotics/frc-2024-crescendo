// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.individual;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionAutoIntake extends Command {
  SwerveSubsystem drive;
  IntakeShooterSubsystem intake;
  PIDController pidZ, pidY;
  double tolerance, speed, disableTime, ir, time;
  PhotonCamera cam;
  Timer timer;

  /** Creates a new LiningUp. */
  public VisionAutoIntake(SwerveSubsystem DriveTrain, IntakeShooterSubsystem intake, double Tolerance, double speed, double disableTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = DriveTrain;
    this.intake = intake;
    this.tolerance = Tolerance;
    this.speed = speed;
    this.disableTime = disableTime;

    this.cam = Vision.intakeCam;

    addRequirements(drive, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidZ = new PIDController(0.05, 0, 0);
    pidY = new PIDController(0.05, 0, 0);
    pidZ.setTolerance(tolerance);
    pidY.setTolerance(tolerance);

    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    time = timer.get();
    ir = intake.getSensor().getProximity();

    PhotonPipelineResult results = cam.getLatestResult();

    if (results.hasTargets()){   
      PhotonTrackedTarget target = results.getBestTarget();

      drive.drive(new ChassisSpeeds(
        pidY.calculate(target.getPitch(), Constants.Vision.INTAKE_SETPOINT),
       0.0,
        pidZ.calculate(target.getYaw(), 0)
      ));
    } else {
      drive.drive(new ChassisSpeeds(speed, 0.0, 0.0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(new ChassisSpeeds (0.0,0.0,0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ir >= Constants.Intake.NOTE_EDGE || time >= disableTime;
  }
}
