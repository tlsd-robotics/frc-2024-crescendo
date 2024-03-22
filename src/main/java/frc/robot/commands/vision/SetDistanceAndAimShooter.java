// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import TLsdLibrary.Controllers.T16000M;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SetDistanceAndAimShooter extends Command {
  SwerveSubsystem drive;
  PIDController pidX, pidZ;
  T16000M joy;
  double tolerance;
  PhotonCamera cam;

  /** Creates a new LiningUp. */
  public SetDistanceAndAimShooter(SwerveSubsystem DriveTrain, T16000M joy, double Tolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = DriveTrain;
    this.joy = joy;
    this.tolerance = Tolerance;

    this.cam = Vision.shooterCam;

    addRequirements(DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidZ = new PIDController(0.075, 0, 0);
    pidZ.setTolerance(tolerance);

    pidX = new PIDController(0.075, 0, 0);
    pidX.setTolerance(tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult results = cam.getLatestResult();

    if (results.hasTargets()){   
      PhotonTrackedTarget target = Vision.filterResults(results);
      
      drive.driveFieldOriented(new ChassisSpeeds(
        (Math.abs(target.getPitch()) > tolerance) ? pidX.calculate(target.getPitch(), Constants.Vision.OPTIMAL_SHOOTER_DISTANCE) : 0.0,
        MathUtil.applyDeadband(-joy.getRawX(), OperatorConstants.X_DEADBAND),
        (Math.abs(target.getYaw()) > tolerance) ? pidZ.calculate(target.getYaw(), 0) : 0.0
      ));
      
      SmartDashboard.putNumber("Angle from Area: ", Vision.getAngleFromArea(target.getArea()));

      } else{
      drive.driveFieldOriented(new ChassisSpeeds(
        MathUtil.applyDeadband(-joy.getRawY(), OperatorConstants.Y_DEADBAND),
        MathUtil.applyDeadband(-joy.getRawX(), OperatorConstants.X_DEADBAND),
        MathUtil.applyDeadband(-joy.getRawZ(), OperatorConstants.Z_DEADBAND)
      ));
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
    return false;
  }
}