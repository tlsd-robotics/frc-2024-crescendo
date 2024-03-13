// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.individual;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToShooterAngle extends Command {
  /** Creates a new DefaultArmCommand. */

    ArmSubsystem arm;   
    PhotonCamera cam;

    public ArmToShooterAngle(ArmSubsystem arm) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.arm = arm;
        addRequirements(arm);
    }   
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      var results = Vision.shooterCam.getLatestResult();
      var target = results.getBestTarget();

      if (results.hasTargets()) {
        arm.setAngle(Vision.getDistanceFromTag(target));
      }
    }   
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { 
    }   
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      if (interrupted) {
        arm.setAngle(arm.getEncoderAngle()); //Stop the arm in it's current location if the command is interrupted.
      }
    } 
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return arm.armAtSetpoint();
    }
}