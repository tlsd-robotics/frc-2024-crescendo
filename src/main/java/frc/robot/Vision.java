package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*

    Hello from the past. I have realized I may have been over complicating the entire thing. 

    I decided to hop onto limelight's docs because I knew they had some pages on distance estimation. https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance

    Area%..... We could just use the area % of the target to estimate a general distance and I know for a fact that was working.

    Only issue I can possibily think of is if the camera is facing upwards at a harsh enough angle, it may read the bounding box as shorter, therefore decreasing the area value.
    But based on what we were seeing today, I do not think that will cause a noticable issue. 

    And since you said the arm movement will be independant of the targeting buttons, it doesn't make much sense to keep the "distance" gathering code in the command anymore.
    Regardless, Imma keep it there so I can setup the area based junk so you can just copy paste it into the arm commands if you want to try it out.

    And aparently I had mocked up a ArmToShooterAngle command already in the auto/individual/ folder so you can use that and just change stuff around. 

    Changes that will be made:

    - Area based Arm angle calculator. In reality this doesn't even have a function, you will just plug target.getArea() straight into the angle calculator
    - Going to build a filtering system to only get the center tags for the speaker. It will still allow you to align with every other tag thou just in case they want to. 
        * If you want to remove the ability to target anything other than the speakers due to how the arm control will be set up, change the filter conditions.

    - AimShooter and AutoAimShooter will be changed to use the new filtered ID tracking. 

    =========== OLD =============
    =============================
    PhotonTrackedTarget target = results.getBestTarget();

    =========== New =============
    =============================
    PhotonTrackedTarget target = Vision.filterResults(results);


    And the new area based angle control will look something like this

    arm.setAngle(Vision.getAngleFromArea(target.getArea());


    Other than that, I will also be making two more vision commands. 

    SetDistanceAndAimShooter()
    AutoSetDistanceAndAimShooter()

    Pretty much going to be those drive to (0,0) basic pid control commands I mentioned if you want to get some targeting up and running without any testing of the area stuff.
    You will need to either calibrate the camera's crosshair to make 0,0 at the proper shooting distance, or change the setpoint in Constants.Vision


    And here is the link to the desmos regression calc I was using to get the equation. its the one that had a "proper" curve.
    https://www.desmos.com/calculator/ms0xampqze

    Just got to retest a few of the values to rebuild the variables and you should be good to go.

    Best of luck at the competition and hope you guys have fun! -josh
 */

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
    // public static double getDistanceFromTag(PhotonTrackedTarget target) {
    //     // First calculate range
    //     return PhotonUtils.calculateDistanceToTargetMeters(
    //         robotToCam.getTranslation().getZ(),
    //         aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getX(),
    //         robotToCam.getRotation().getY(),
    //         Units.degreesToRadians(target.getPitch())
    //     );

    // }

    // public static double getDistancePoseTest(PhotonTrackedTarget target) {
    //     return PhotonUtils.getDistanceToPose(cameraPose, aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().toPose2d());
    // }

    // public static double getDistanceTesting(PhotonTrackedTarget target) {
    //     return (1/(57.125 - 6)) * (Math.tan(Units.degreesToRadians(target.getPitch()) + Units.degreesToRadians(15)));
    // }

    //======================== Calculate Distance from Target ===========================
    // public static double getDistanceFromTarget(PhotonTrackedTarget target) {
    //     Transform3d t = target.getBestCameraToTarget();

    //     return Math.sqrt(
    //         Math.pow(t.getX(), 2) +
    //         Math.pow(t.getY(), 2) +
    //         Math.pow(t.getZ(), 2)
    //     );
    // }

    // public static double getAngleFromDistance(double distance) {
    //     double a = -274.818;
    //     double k = -1.51204;
    //     double d = -1.3563;
    //     double c = -49.8021;

    //    return a * (Math.pow(2, (k * (distance - d)))) + c;
    // }


    // Literal copy paste of the method above... but I don't wanna delete stuff rn. Comments will work just fine.
    // Going to have to get all these variables again using the new area values, but that doesn't take too long in actuality
    public static double getAngleFromArea(double area) {
        double a = -274.818;
        double k = -1.51204;
        double d = -1.3563;
        double c = -49.8021;

       return a * (Math.pow(2, (k * (area - d)))) + c;
    }

    /**
     * Absolute VILE looking line, but pretty much get a list of targets from the result, then makes a stream for the list.
     * That stream is then filtered to only include those that don't equal 3 or 8 (The offset speaker tags)
     * Then grab the first element in the list, which is sorted based on the sorting mode (default is largest). So it returns the largest target that isn't 3 or 8.
     * And if you need to, you can chain multiple .filter()'s together
     * @param result
     * @return Largest Target not on the ignore list
     */
    public static PhotonTrackedTarget filterResults(PhotonPipelineResult result) {
        return result.getTargets().stream().filter(t -> t.getFiducialId() != 3 && t.getFiducialId() != 8).findFirst().get();
    }
}
