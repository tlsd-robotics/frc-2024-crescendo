// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.individual;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignWithNote extends Command {
  SwerveSubsystem drive;
  PIDController pidZ;
  double tolerance;
  double spinRate;
  PhotonCamera cam;

  /** Creates a new LiningUp. */
  public AlignWithNote(SwerveSubsystem DriveTrain, double Tolerance, double spinRate) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = DriveTrain;
    this.tolerance = Tolerance;
    this.spinRate = Units.degreesToRadians(spinRate);

    this.cam = Vision.intakeCam;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidZ = new PIDController(0.05, 0, 0);
    pidZ.setTolerance(tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult results = cam.getLatestResult();

    if (results.hasTargets()){   
      PhotonTrackedTarget target = results.getBestTarget();

      drive.drive(new ChassisSpeeds(0.0, 0.0, pidZ.calculate(target.getYaw(), 0)));
    } else {
      drive.drive(new ChassisSpeeds(0.0, 0.0, spinRate));
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
    return pidZ.atSetpoint();
  }
}
