package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public  class Vision {
    //================== Cameras ========================
    public static PhotonCamera shooterCam = new PhotonCamera("TopUsb");
    public static PhotonCamera intakeCam = new PhotonCamera("BottomUsb");

    //==================== Targets ==========================
    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    //==================== Pose Estimation ==========================
    public static Transform3d robotToCam = new Transform3d(Constants.Vision.ROBOT_TO_CAM_TRANSLATION, Constants.Vision.ROBOT_TO_CAM_ROTATION); // In meters and radians
    public static PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, shooterCam, robotToCam);

    //================= Return Estimated Pose =================
    public static Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    //======================== Calculate Distance from Apriltag ===========================
    public static double getDistanceFromTag(PhotonTrackedTarget target) {
        double distance = 0.0;

        // First calculate range
        distance = PhotonUtils.calculateDistanceToTargetMeters(
            robotToCam.getTranslation().getZ(),
            aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getTranslation().getZ(),
            robotToCam.getRotation().getY(),
            Units.degreesToRadians(target.getPitch())
        );

        return distance;

    }
}
