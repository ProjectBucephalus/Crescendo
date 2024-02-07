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

        s_Swerve.visionDrive(translation, (calculateRequiredHeading().getRadians()) * 60, true, brakeVal);
        s_Pivot.setPosition(PivotPosition.SPEAKER);
        s_Pivot.moveArmToAngle(calculatedRequiredShooterAngle() + 28);
        SmartDashboard.putNumber("robot pose heading", calculateRequiredHeading().getDegrees());
        SmartDashboard.putNumber("calculated shooter angle", calculatedRequiredShooterAngle());
    }

    @Override
    public void end(boolean end) {
        s_Swerve.setVisionAlignmentBool(false);
        //s_Pivot.setPosition(PivotPosition.STOWED);
    }

    public Rotation2d calculateRequiredHeading() {
        var pose = s_Swerve.getEstimatedPose();
        //TODO Change with alliances
        return PhotonUtils.getYawToPose(pose, new Pose2d(0, 5.6, new Rotation2d(0, 0)));
    }

    public double calculatedRequiredShooterAngle() {
        var pose = s_Swerve.getEstimatedPose();
        SmartDashboard.putNumber("distance to target", PhotonUtils.getDistanceToPose(pose, new Pose2d(0.2, 5.6, new Rotation2d(0, 0))));
        return Units.radiansToDegrees(Math.atan(2.03/ (PhotonUtils.getDistanceToPose(pose, new Pose2d(0.2, 5.6, new Rotation2d(0, 0))))));
    }
}
