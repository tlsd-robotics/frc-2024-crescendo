// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;
import TLsdLibrary.Controllers.T16000M;
import TLsdLibrary.Vision.Limelight;
import TLsdLibrary.Vision.Target;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class LiningUp extends Command {
  SwerveSubsystem drive;
  Limelight cam;
  Target target;
  PIDController pidX, pidZ;
  T16000M joy;



  /** Creates a new LiningUp. */
  public LiningUp(SwerveSubsystem DriveTrain, Limelight cam, Target target, T16000M joy ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = DriveTrain;
    this.cam = cam;
    this.target = target;
    this.joy = joy;

    addRequirements(DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidX = new PIDController(0.05, 0, 0);
    pidZ = new PIDController(0.05, 0, 0);
    pidX.setTolerance(target.getTolerance());
    pidZ.setTolerance(target.getTolerance());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (cam.getIsTargetFound()){      
      drive.drive(new ChassisSpeeds(
        MathUtil.applyDeadband(-joy.getRawY(), OperatorConstants.Y_DEADBAND),
        // (!pidX.atSetpoint()) ? pidX.calculate(cam.getHorizontalError(), 0) : 0.0, 
        MathUtil.applyDeadband(-joy.getRawX(), OperatorConstants.X_DEADBAND),
        pidZ.calculate(cam.getHorizontalError(), 0)
      ));

      SmartDashboard.putNumber("Distance", cam.getDistanceToTarget(target));
    } else{
      drive.drive(new ChassisSpeeds(
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
