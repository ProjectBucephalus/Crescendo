package frc.robot.VisionCommands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Pivot.PivotPosition;

public class AimToSpeakerNoDrive extends Command {

    public Swerve s_Swerve;
    public Pivot s_Pivot;

    public AimToSpeakerNoDrive(Swerve s_Swerve, Pivot s_Pivot) {
        this.s_Swerve = s_Swerve;
        this.s_Pivot = s_Pivot;
        SmartDashboard.putNumber("robot pose heading", calculateRequiredHeading().getDegrees());

    }

    @Override
    public void initialize() {
        s_Swerve.setVisionAlignmentBool(true);
    }

    @Override
    public void execute() {
        Translation2d translation = new Translation2d(0, 0).times(Constants.Swerve.maxSpeed);
        s_Swerve.drive(translation, (calculateRequiredHeading().rotateBy(Rotation2d.fromDegrees(180)).getRadians()) * 70, false, true, 0.0);
        s_Pivot.setPosition(PivotPosition.SPEAKER);
        s_Pivot.setDesiredPostion(-calculatedRequiredShooterAngle());
        SmartDashboard.putNumber("robot pose heading", calculateRequiredHeading().getDegrees());
        SmartDashboard.putNumber("calculated shooter angle", calculatedRequiredShooterAngle());
    }

    public Rotation2d calculateRequiredHeading() {
        var pose = s_Swerve.getEstimatedPose();
        //TODO Change with alliances
        return PhotonUtils.getYawToPose(pose, new Pose2d(0, 5.6, new Rotation2d(0, 0)));
    }

    public double calculatedRequiredShooterAngle() {
        // var pose = s_Swerve.getEstimatedPose();
        // SmartDashboard.putNumber("distance to target", PhotonUtils.getDistanceToPose(pose, new Pose2d(0.2, 5.6, new Rotation2d(0, 0))));
        // return Units.radiansToDegrees(Math.atan((1.95-0.425) / (PhotonUtils.getDistanceToPose(pose, new Pose2d(0.2, 5.6, new Rotation2d(0, 0))))));
        return -75;
    }
    @Override
    public boolean isFinished() {
        if (Math.abs(calculateRequiredHeading().getDegrees()) < 5){
            s_Swerve.setVisionAlignmentBool(false);
            return true;
        } else {
            return false;
        }
    }
}
