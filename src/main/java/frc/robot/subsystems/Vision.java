package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * vision subsystem
 * @authour 5985
 */
public class Vision extends SubsystemBase {

    public PhotonCamera leftCam;
    public PhotonCamera rightCam;
    public PhotonCamera frontCam;
    public Swerve s_Swerve;

    public PhotonPoseEstimator photonEstimator;

    public Vision(PhotonCamera leftCam, PhotonCamera rightCam, PhotonCamera frontCam, Swerve s_swerve) {
        this.leftCam = leftCam;
        this.rightCam = rightCam;
        this.frontCam = frontCam;
        this.s_Swerve = s_swerve;
        photonEstimator = new PhotonPoseEstimator(null, null, null);
        SmartDashboard.putBoolean("Using Vision", false);
    }

    /**
     * Checks if there are any avaliable apriltags and updates the swerve odometry.
     */
    public void updateSwervePose() {
        var leftResult = leftCam.getLatestResult();
        var rightResult = rightCam.getLatestResult();
        var frontResult = frontCam.getLatestResult();
        if (Constants.useVision == true) {
            if (!checkAndUpdateResult(leftResult, Constants.Vision.leftCamToRobot)) {
                if (!checkAndUpdateResult(rightResult, Constants.Vision.rightCamToRobot)) {
                    if (!checkAndUpdateResult(frontResult, Constants.Vision.frontCamToRobot)) {
                        SmartDashboard.putBoolean("Using Vision", false);
                    }
                }
            }
        }

    }

    public boolean checkAndUpdateResult(PhotonPipelineResult cam, Transform3d camToRobot) {
        if (cam.getMultiTagResult().estimatedPose.isPresent) {
            Transform3d fieldToCamera = cam.getMultiTagResult().estimatedPose.best;
            var best = new Pose3d()
                    .plus(camToRobot.inverse())
                    .plus(fieldToCamera);
            Rotation3d rotation = best.getRotation();

            Pose2d pose = new Pose2d(
                    best.getX(), best.getY(),
                    new Rotation2d(rotation.getX(), rotation.getY()));

            // new EstimatedRobotPose(
            // best,
            // result.getTimestampSeconds(),
            // result.getTargets(),
            // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);

            s_Swerve.resetEstimatedOdometry(pose);

            // System.out.println(fieldToCamera);
            SmartDashboard.putBoolean("Using Vision", true);

            return true;
        } else {
            return false;
        }
    }

}
