package frc.robot.VisionCommands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.SwerveConstants;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Pivot.PivotPosition;
import frc.robot.subsystems.Shooter.ShooterState;

public class aimToSpeaker extends Command {

    public Swerve s_Swerve;
    public Pivot s_Pivot;
    public Shooter s_Shooter;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier brakeSup;

    public aimToSpeaker(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier brakeSup, Pivot s_Pivot, Shooter s_Shooter) {
        this.s_Swerve = s_Swerve;
        this.s_Pivot = s_Pivot;
        this.s_Shooter = s_Shooter;
        SmartDashboard.putNumber("robot pose heading", calculateRequiredHeading().getDegrees());

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.brakeSup = brakeSup;

        SmartDashboard.putNumber("Pivot position for array", 0);
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
        Translation2d translation = new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed);

        s_Swerve.visionDrive(translation,
                (calculateRequiredHeading().rotateBy(Rotation2d.fromDegrees(180)).getRadians()) * 70, true, brakeVal);

        /* Used for figuring out how we should shoot */
        s_Pivot.setPosition(PivotPosition.SPEAKER);
        s_Shooter.setShooterState(ShooterState.RUNNING);

        s_Pivot.updateSpeakerAngle();

        // the -4 is purely for backlash adjustment
        // s_Pivot.setDesiredPostion(calculatedRequiredShooterAngle());
        // s_Pivot.setDesiredPostion(SmartDashboard.getNumber("Pivot position for
        // array", 0));

        s_Swerve.setWithinRequiredHeading(Math.abs(s_Swerve.getEstimatedPose().getRotation().getDegrees()
                - Math.abs(calculateRequiredHeading().rotateBy(Rotation2d.fromDegrees(180))
                        .getDegrees())) < SwerveConstants.ANGLE_TOLERANCE_DEGREES);
        SmartDashboard.putNumber("Is our auto aligned heading aligned?",
                Math.abs(s_Swerve.getEstimatedPose().getRotation().getDegrees()
                        - Math.abs(calculateRequiredHeading().rotateBy(Rotation2d.fromDegrees(180)).getDegrees())));
        SmartDashboard.putNumber("robot pose heading", calculateRequiredHeading().getDegrees());
        // SmartDashboard.putNumber("calculated shooter angle", calculatedRequiredShooterAngle());

    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean end) {
        s_Swerve.setVisionAlignmentBool(false);
        s_Pivot.setPosition(PivotPosition.STOWED);
        s_Shooter.setShooterState(ShooterState.IDLE);

        /*
         * Make sure we do this so that other manual alignment functions work. It should
         * only be false if we are currently auto aligning and not withing alignemnt
         * tolerance.
         */
        s_Swerve.setWithinRequiredHeading(true);
    }

    public Rotation2d calculateRequiredHeading() {
        var pose = s_Swerve.getEstimatedPose();
        return PhotonUtils.getYawToPose(pose,
                FieldConstants.flipPose(new Pose2d(FieldConstants.SPEAKER, new Rotation2d(0, 0))));
    }

    
    
}
