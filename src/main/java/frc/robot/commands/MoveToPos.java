package frc.robot.commands;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Swerve;

/**
 * PointToAngle Command
 * @author 5985
 * @author Alec
 * @author Aidan
 */
public class MoveToPos extends Command {
    private frc.robot.subsystems.Swerve s_Swerve;
    private Rotation2d targetRotation;
    private Translation2d targetTranslation;
    private Pose2d currentPose, startPose, endPose;
    private boolean useTranslation;

    /**
     * Sets a target angle for the robot to face, intake forward
     * @param targetRotation Degrees, field-relative, for robot intake to face
     * @author 5985
     * @author Alec
     */
    public MoveToPos(Rotation2d targetRotation, Translation2d targetTranslation, frc.robot.subsystems.Swerve s_Swerve, boolean useTranslation)
    {
        this.s_Swerve = s_Swerve;
        this.targetRotation = targetRotation;
        this.useTranslation = useTranslation;
        this.targetTranslation = targetTranslation;
    }

    @Override
    public void execute() 
    {
        if (!useTranslation) 
        {
            targetTranslation = s_Swerve.getEstimatedPose().getTranslation();
        }
        
        currentPose = s_Swerve.getEstimatedPose();

        // The rotation component in these poses represents the direction of travel
        //Pose2d startPos = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());
        startPose = currentPose;
        endPose = new Pose2d(targetTranslation, targetRotation);

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPose, endPose);
        PathPlannerPath path = new PathPlannerPath
        (
            bezierPoints,
            new PathConstraints(
            4.0, 4.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540)),
            new GoalEndState(0.0, endPose.getRotation())
        );

        // Prevent this path from being flipped on the red alliance, since the given
        // positions are already correct
        path.preventFlipping = true;

        AutoBuilder.followPath(path).schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
