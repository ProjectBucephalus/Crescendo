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
 * 
 * @author 5985
 * @author Aidan
 */
public class Vision extends SubsystemBase 
{
    // The camera objects
    public PhotonCamera leftCam;
    public PhotonCamera rightCam;
    public PhotonCamera frontCam;

    // An instance of the swerve subsystem
    public Swerve s_Swerve;

    // A PhotonPoseEstimator, which estimates the robot's posiiton on the field based on AprilTag inputs
    public PhotonPoseEstimator photonEstimator;

    public Vision(PhotonCamera leftCam, PhotonCamera rightCam, PhotonCamera frontCam, Swerve s_swerve) 
    {
        this.leftCam = leftCam;
        this.rightCam = rightCam;
        this.frontCam = frontCam;
        this.s_Swerve = s_swerve;
        
        photonEstimator = new PhotonPoseEstimator(null, null, null);
        SmartDashboard.putBoolean("Using Vision", false);
    }

    /**
     * Checks if there are any avaliable apriltags and updates the swerve odometry.
     * @author 5985
     * @author Aidan
     */
    public void updateSwervePose() 
    {
        var leftResult = leftCam.getLatestResult();
        var rightResult = rightCam.getLatestResult();
        var frontResult = frontCam.getLatestResult();
        if (Constants.useVision == true) 
        {
            if (!checkAndUpdateResult(frontResult, Constants.Vision.frontCamToRobot)) 
            {
                SmartDashboard.putBoolean("Using Vision", false);
            }
        }
    }

    /**
     * 
     * @param cam A pipeline result object
     * @param camToRobot The offset from the input camera to the centre of the robot
     * @return Whether it successfully updated the position
     * @author 5985
     * @author Aidan
     */
    public boolean checkAndUpdateResult(PhotonPipelineResult cam, Transform3d camToRobot) 
    {
        if (cam.getMultiTagResult().estimatedPose.isPresent) // Runs if the estimated pose from the input cam is valid and exists
        {
            // Defines a Transform3d to represent the field-relative position of the input camera, 
            // then sets it to the best pose estimation from the camera result
            Transform3d fieldToCamera = cam.getMultiTagResult().estimatedPose.best; 
            
            // Creates a Pose3d equal to the inverse of the input camera offset, plus the field relative position of the camera
            var best = new Pose3d()
                    .plus(camToRobot.inverse())
                    .plus(fieldToCamera);
            
            // Creates a Rotation3d equal to the rotation of the above Pose3d
            Rotation3d rotation = best.getRotation();

            // Creates a Pose2d with the x and y values of the above Pose3d, and the rotation values of the above Rotation3d
            Pose2d pose = new Pose2d(
                    best.getX(), best.getY(),
                    new Rotation2d(rotation.getX(), rotation.getY()));

            // Resets the swerve odometry to the above Pose2d
            s_Swerve.resetEstimatedOdometry(pose);

            SmartDashboard.putBoolean("Using Vision", true);

            return true;
        } 
        else 
        {
            return false;
        }
    }

}
