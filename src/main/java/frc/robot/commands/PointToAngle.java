package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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

    private PIDController pid = new PIDController(0.1, 0, 2);

    public PointToAngle(Swerve s_Swerve, Transform2d target) {
        this.s_Swerve = s_Swerve;
        targetRotation = target.getRotation().getDegrees();

    }

    @Override
    public void execute() {
        // var turningVal =
        // pid.calculate(s_Swerve.getEstimatedPose().getRotation().getDegrees(),
        // targetRotation);

        // TODO look at the tuning of this. Maybe look at implementing a pid controller.
        // s_Swerve.driveRobotRelative(new ChassisSpeeds(0, 0,
        // pid.calculate(s_Swerve.getEstimatedPose().getRotation().getDegrees(), 90)));
        // s_Swerve.driveRobotRelative(new ChassisSpeeds(0, 0,(calculateRequiredHeading()/10)));
        s_Swerve.driveRobotRelative(new ChassisSpeeds(0, 0,-calculateRequiredHeading()/10));
    }

    @Override
    public void initialize() {
        pid.reset();
        s_Swerve.setVisionAlignmentBool(true);
    }

    @Override
    public void end(boolean end) {
        s_Swerve.setVisionAlignmentBool(false);
    }

    public double calculateRequiredHeading() {
        var pose = s_Swerve.getEstimatedPose();
        // flip the pose so the alignments work on the other side of the field.
        double requiredHeading = pose.getRotation().getDegrees() - targetRotation;
        
        // Adjust the required heading to keep it within a reasonable range
        if (requiredHeading > 180) {
            requiredHeading -= 360; // Keep the heading within -180 to 180 degrees
        } else if (requiredHeading < -180) {
            requiredHeading += 360; // Keep the heading within -180 to 180 degrees
        }
        
        return requiredHeading;
    }
    

    @Override
    public boolean isFinished() {
        // return (Math.abs(s_Swerve.getEstimatedPose().getRotation().getDegrees()
        //         - Math.abs(calculateRequiredHeading())) < SwerveConstants.ANGLE_TOLERANCE_DEGREES);
        return false;
    }
}
