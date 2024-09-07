package frc.robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;


public class Vision {
    public static final double kVisionNoTarget = 2000062;

    public static final double kCameraHeight = 0; // Change when the numbers are known
    public static final double kTargetHeight = 0;
    public static final double kCameraPitch = 0;
    public static final double kTargetPitch = 0;
    
    public static final double targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters
    public static final double targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters
    public static final double kFarTgtXPos = Units.feetToMeters(54);
    public static final double kFarTgtYPos = Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
    public static final double kFarTgtZPos = (Units.inchesToMeters(98.19) - targetHeight) / 2 + targetHeight;
    public static final Pose3d kFarTargetPose = new Pose3d( new Translation3d(kFarTgtXPos, kFarTgtYPos, kFarTgtZPos), new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)));

    private PhotonCamera photonCamera;
    public Vision(PhotonCamera camera){
        photonCamera = camera;
    }

    PhotonCamera camera = new PhotonCamera("cam1");


    private PhotonPipelineResult getResult(){
        return photonCamera.getLatestResult();
    }
    // Gives you the target from an ID.
    private PhotonTrackedTarget getTargetFromID(Integer ID){
        if(hasTargets()){
            var results = getResult();
            var targets = results.getTargets();
            for(int i = 0; i< targets.size(); i++){
                var target = targets.get(i);
                if(target.getFiducialId() == ID){
                    return target;
                }
            }
            return null;
        }else{
            return null;
        }
    }
    
    public boolean hasTargets(){
        var results = getResult();
        return results.hasTargets();
    }
    // Gives you the yaw from the best target.
    public double getYaw(){
        var result = getResult();
        if(hasTargets()){
            var target = result.getBestTarget();
            return target.getYaw();
        }else{
            return kVisionNoTarget;
        }
    }
    // Gives you the yaw from the target.
    public double getYaw(Integer tagNumber){
        if(hasTargets()){
            var target = getTargetFromID(tagNumber);
            if(target == null){
                return kVisionNoTarget;
            }
            return target.getYaw();
        }else{

            return kVisionNoTarget;
        }
    }
    // Gives you the pitch from the best target.
    public double getPitch(){
        var result = getResult();
        if(hasTargets()){
            var target = result.getBestTarget();
            return target.getPitch();
        }else{
            return kVisionNoTarget;
        }
    }
    // Gives you the pitch from the target.
    public double getPitch(Integer tagNumber){
        if(hasTargets()){
            var target = getTargetFromID(tagNumber);
            if(target == null){
                return kVisionNoTarget;
            }
            return target.getPitch();
        }else{

            return kVisionNoTarget;
        }
    }
    // Gives you the roll from the best target.
    public double getRoll(){
        var result = getResult();
        if(hasTargets()){
            var target = result.getBestTarget();
            return target.getSkew();
        }else{
            return kVisionNoTarget;
        }
    }
    // Gives you the roll from the target.
    public double getRoll(Integer tagNumber){
        if(hasTargets()){
            var target = getTargetFromID(tagNumber);
            if(target == null){
                return kVisionNoTarget;
            }
            return target.getSkew();
        }else{

            return kVisionNoTarget;
        }
    }

    // public double getXAndYPose(){
    //     var result = getResult();
    //     var target = result.getBestTarget();
    //     var cameraToRobot = new Transform2d(new Translation2d(0,0), new Rotation2d(0,0));
    //     Pose2d targetPose = target.getCameraToTarget();
    //     Pigeon2 gyro = new Pigeon2(0);
    //     if (hasTargets()) {
    //         Pose2d robotPose = PhotonUtils.estimateFieldToRobot(kCameraHeight, kTargetHeight, kCameraPitch, kTargetPitch, Rotation2d.fromDegrees(-target.getYaw()), gyro.getRotation2d(), targetPose, cameraToRobot);
    //         AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    //     }else{
    //         return kVisionNoTarget;
    //     }
    // }

    // public double distanceFromTarget(){
    //     var result = camera.getLatestResult();
    //     if(hasTargets()) {
    //         double range = PhotonUtils.calculateDistanceToTargetMeters(0.0, 0.0, 0.0, Units.degreesToRadians(result.getBestTarget().getPitch()));
    //     }
    // }






}

