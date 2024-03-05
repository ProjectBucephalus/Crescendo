package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Swerve;

/**
 * PointToAngle Command
 * 
 * @author Alec
 * @author Aidan
 * @author 5985
 */
public class PointToAngle extends Command {
    private Swerve s_Swerve;
    private double targetRotation;

    private PIDController pid = new PIDController(10, 2, 2);

    public PointToAngle(Swerve s_Swerve, Transform2d target) {
        this.s_Swerve = s_Swerve;
        targetRotation = target.getRotation().getDegrees();

    }

    @Override
    public void execute() {
        var turningVal = pid.calculate(calculateRequiredHeading().getDegrees());

        // TODO look at the tuning of this. Maybe look at implementing a pid controller.
        s_Swerve.driveRobotRelative(new ChassisSpeeds(0, 0, turningVal));
    }

    @Override
    public void initialize() {
        s_Swerve.setVisionAlignmentBool(true);
    }

    @Override
    public void end(boolean end) {
        s_Swerve.setVisionAlignmentBool(false);
    }

    public Rotation2d calculateRequiredHeading() {
        var pose = s_Swerve.getEstimatedPose();
        // flip the pose so the alignments work on the other side of the field.
        return PhotonUtils.getYawToPose(pose,
                FieldConstants.flipPose(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(targetRotation))));
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(s_Swerve.getEstimatedPose().getRotation().getDegrees()
                - Math.abs(calculateRequiredHeading().getDegrees())) < Constants.Swerve.ANGLE_TOLERANCE_DEGREES);
    }
}
