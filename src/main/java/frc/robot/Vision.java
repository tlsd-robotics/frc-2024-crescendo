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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public  class Vision {
    //================== Cameras ========================
    public static PhotonCamera shooterCam = new PhotonCamera("TopUsb");
    public static PhotonCamera intakeCam = new PhotonCamera("BottomUsb");

    //==================== Targets ==========================
    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    //==================== Pose Estimation ==========================
    public static Transform3d robotToCam = new Transform3d(Constants.Vision.ROBOT_TO_CAM_TRANSLATION, Constants.Vision.ROBOT_TO_CAM_ROTATION); // In meters and radians
    public static PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, shooterCam, robotToCam);
    public static Pose2d cameraPose = new Pose2d(Constants.Vision.ROBOT_TO_CAM_TRANSLATION.toTranslation2d(), Constants.Vision.ROBOT_TO_CAM_ROTATION.toRotation2d());

    //================= Return Estimated Pose =================
    public static Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    //======================== Calculate Distance from Apriltag ===========================
    public static double getDistanceFromTag(PhotonTrackedTarget target) {
        // First calculate range
        return PhotonUtils.calculateDistanceToTargetMeters(
            robotToCam.getTranslation().getZ(),
            aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getX(),
            robotToCam.getRotation().getY(),
            Units.degreesToRadians(target.getPitch())
        );

    }

    public static double getDistancePoseTest(PhotonTrackedTarget target) {
        return PhotonUtils.getDistanceToPose(cameraPose, aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().toPose2d());
    }

    public static double getDistanceTesting(PhotonTrackedTarget target) {
        return (1/(57.125 - 6)) * (Math.tan(Units.degreesToRadians(target.getPitch()) + Units.degreesToRadians(15)));
    }

    //======================== Calculate Distance from Target ===========================
    public static double getDistanceFromTarget(PhotonTrackedTarget target) {
        Transform3d t = target.getBestCameraToTarget();

        return Math.sqrt(
            Math.pow(t.getX(), 2) +
            Math.pow(t.getY(), 2) +
            Math.pow(t.getZ(), 2)
        );
    }

    public static double getAngleFromDistance(double distance) {
        double a = -274.818;
        double k = -1.51204;
        double d = -1.3563;
        double c = -49.8021;

       return a * (Math.pow(2, (k * (distance - d)))) + c;
    }
}
