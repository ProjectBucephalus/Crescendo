package frc.robot.VisionCommands;

import java.util.Arrays;
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
import frc.robot.FieldConstants;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Pivot.PivotPosition;

public class aimToSpeaker extends Command {

    public Swerve s_Swerve;
    public Pivot s_Pivot;
    private AprilTagFieldLayout fieldLayout;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier brakeSup;

    public aimToSpeaker(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier brakeSup, Pivot s_Pivot) {
        this.s_Swerve = s_Swerve;
        this.s_Pivot = s_Pivot;
        SmartDashboard.putNumber("robot pose heading", calculateRequiredHeading().getDegrees());

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.brakeSup = brakeSup;

    }

    @Override
    public void initialize() {
        s_Swerve.setVisionAlignmentBool(true);

    }

    @Override
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double brakeVal = MathUtil.applyDeadband(brakeSup.getAsDouble(), Constants.stickDeadband);
        Translation2d translation = new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed);


        s_Swerve.visionDrive(translation,
                (calculateRequiredHeading().rotateBy(Rotation2d.fromDegrees(180)).getRadians()) * 70, true, brakeVal);

        /* Used for figuring out how we should shoot */
        s_Pivot.setPosition(PivotPosition.SPEAKER);

        // the -4 is purely for backlash adjustment
        s_Pivot.setDesiredPostion(calculatedRequiredShooterAngle());

        s_Swerve.setWithinRequiredHeading(Math.abs(s_Swerve.getEstimatedPose().getRotation().getDegrees()
                - Math.abs(calculateRequiredHeading().getDegrees())) < Constants.Swerve.ANGLE_TOLERANCE_DEGREES);
        SmartDashboard.putNumber("robot pose heading", calculateRequiredHeading().getDegrees());
        SmartDashboard.putNumber("calculated shooter angle", calculatedRequiredShooterAngle());

        
    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean end) {
        s_Swerve.setVisionAlignmentBool(false);
        s_Pivot.setPosition(PivotPosition.STOWED);

        /*
         * Make sure we do this so that other manual alignment functions work. It should
         * only be false if we are currently auto aligning and not withing alignemnt
         * tolerance.
         */
        s_Swerve.setWithinRequiredHeading(true);
    }

    public Rotation2d calculateRequiredHeading() {
        var pose = s_Swerve.getEstimatedPose();
        // TODO Change with alliances
        return PhotonUtils.getYawToPose(pose, new Pose2d(FieldConstants.SPEAKER, new Rotation2d(0, 0)));
    }

    public double calculatedRequiredShooterAngle() {

        var pose = s_Swerve.getEstimatedPose();

        double[] distances = {5, 10, 15, 20}; // distances in meters
        double[] angles    = {30, 45, 60, 75};   // shooter angles in degrees

        // SmartDashboard.putNumber("distance to target",
        //         PhotonUtils.getDistanceToPose(pose, new Pose2d(0.2, 5.6, new Rotation2d(0, 0))));
        
        // Example target distance
        double targetDistance = PhotonUtils.getDistanceToPose(pose, new Pose2d(0.2, 5.6, new Rotation2d(0, 0)));
        
        // Ensure the target distance is within the range of the data
        if (targetDistance < distances[0]) {
            // Extrapolate using the first two points
            return angles[0] + (angles[1] - angles[0]) * (targetDistance - distances[0]) / (distances[1] - distances[0]);
        } else if (targetDistance > distances[distances.length - 1]) {
            // Extrapolate using the last two points
            int n = distances.length;
            return angles[n - 2] + (angles[n - 1] - angles[n - 2]) * (targetDistance - distances[n - 2]) / (distances[n - 1] - distances[n - 2]);
        }

        // Perform linear interpolation
        double angle = 0;
        for (int i = 0; i < distances.length - 1; i++) {
            if (targetDistance >= distances[i] && targetDistance <= distances[i + 1]) {
                angle = angles[i] + (angles[i + 1] - angles[i]) * (targetDistance - distances[i]) / (distances[i + 1] - distances[i]);
                break;
            }
        }
        return angle;

        // return Units.radiansToDegrees(Math.atan(
        //         (2 - 0.425) / (PhotonUtils.getDistanceToPose(pose, new Pose2d(0.2, 5.6, new Rotation2d(0, 0))))));
        // return 0;
    }
}
