// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.individual;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoAimShooter extends Command {
  SwerveSubsystem drive;
  IntakeShooterSubsystem shooter;
  PIDController pidY, pidZ;
  double tolerance, distance;
  PhotonCamera cam;
  boolean safeDistance;

  /** Creates a new LiningUp. */
  public AutoAimShooter(SwerveSubsystem DriveTrain, IntakeShooterSubsystem shooter, double Tolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = DriveTrain;
    this.tolerance = Tolerance;
    this.shooter = shooter;

    this.cam = Vision.shooterCam;

    addRequirements(DriveTrain, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidZ = new PIDController(0.05, 0, 0);
    pidY = new PIDController(0.05, 0, 0);
    pidZ.setTolerance(tolerance);
    pidY.setTolerance(tolerance);

    safeDistance = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult results = cam.getLatestResult();

    if (results.hasTargets()){   
      PhotonTrackedTarget target = results.getBestTarget();
      distance = Vision.getDistanceFromTag(target);
      
      drive.drive(new ChassisSpeeds(
        getPidDistanceValue(distance), 
        0.0,
        pidZ.calculate(target.getYaw(), 0)
      ));

      SmartDashboard.putNumber("Distance", distance);
    }
  }

  public double getPidDistanceValue(double distance) {
    if (distance > Constants.Shooter.MAX_RANGE) {
      return pidY.calculate(distance, Constants.Shooter.MAX_RANGE - 0.5);
    } else {
      safeDistance = true;

      return 0.0;
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
    return pidZ.atSetpoint() && safeDistance;
  }
}
