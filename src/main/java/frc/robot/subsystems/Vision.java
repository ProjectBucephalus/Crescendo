package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

    public PhotonCamera frontCam;
    public PhotonCamera backCam;
    public Swerve s_Swerve;

    public PhotonPoseEstimator photonEstimator;

    public Vision(PhotonCamera frontCam, PhotonCamera backCam, Swerve s_swerve) {
        this.frontCam = frontCam;
        this.backCam = backCam;
        this.s_Swerve = s_swerve;
    }
    public void updateSwervePose() {
        var result = frontCam.getLatestResult();
        if (result.getMultiTagResult().estimatedPose.isPresent) {
            Transform3d fieldToCamera = result.getMultiTagResult().estimatedPose.best;
            var best =
                new Pose3d()
                        .plus(Constants.Vision.frontCamToRobot.inverse())
                        .plus(fieldToCamera); 
            Rotation3d rotation = best.getRotation();

            Pose2d pose = new Pose2d(
                best.getX(), best.getY(), 
                new Rotation2d(rotation.getX(), rotation.getY()));

            s_Swerve.resetOdometry(pose);
            
            System.out.println(fieldToCamera); 
        }
        
    }

    
}
