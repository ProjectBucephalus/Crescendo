package frc.robot.commands;

import java.lang.reflect.Field;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Swerve;

/**
 * PointToAngle Command
 * @author Alec
 * @author 5985
 */
public class PointToAngle extends Command {
    private Swerve s_Swerve;
    private double targetRotation;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier brakeSup;

    public PointToAngle(Swerve s_Swerve,  DoubleSupplier translationSup, DoubleSupplier strafeSup,
    DoubleSupplier brakeSup, double targetRotation)
    {
        this.s_Swerve = s_Swerve;
        this.targetRotation = targetRotation;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.brakeSup = brakeSup;
    }

    @Override
    public void execute() 
    {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double brakeVal = MathUtil.applyDeadband(brakeSup.getAsDouble(), Constants.stickDeadband);
        Translation2d translation = new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed);

        //TODO look at the tuning of this. Maybe look at implementing a pid controller. 
        s_Swerve.visionDrive(translation,
                (calculateRequiredHeading().getRadians()) * 70, true, brakeVal);


        s_Swerve.visionDrive(null, targetRotation, isScheduled(), targetRotation);

        
    }

    public Rotation2d calculateRequiredHeading() {
        var pose = s_Swerve.getEstimatedPose();
        // flip the pose so the alignments work on the other side of the field.
        return PhotonUtils.getYawToPose(pose, FieldConstants.flipPose(new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(targetRotation))));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
