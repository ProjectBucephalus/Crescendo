package frc.robot.commands;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.VisionCommands.aimToSpeaker;
import frc.robot.commands.Intake.GetBeamBreak;
import frc.robot.commands.Intake.IntakeAndDeployPivot;
import frc.robot.commands.Shooter.AutoPivotShootSequence;
import frc.robot.commands.Shooter.ShootSequence;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeStatus;
import frc.robot.subsystems.Shooter.ShootPosition;
import frc.robot.subsystems.Shooter.ShooterState;

public class GetCentreNote extends GetNote {    
    private PathPlannerPath m_returnPath;

    private void setReturnPath(Translation2d targetNote) {
        String[] pathnameArray = s_pathLookup.get(targetNote);
        // return path is last item
        if (null != pathnameArray[pathnameArray.length - 1]) {
            m_returnPath = FieldConstants.loadPath(pathnameArray[pathnameArray.length - 1]);
        }
    }

    public GetCentreNote(Translation2d targetNote, Swerve s_Swerve, NoteVision noteVision,
            Shooter s_Shooter, Pivot s_Pivot, Intake s_Intake) {

        super(targetNote, s_Swerve, noteVision, s_Shooter, s_Intake);

        if (!FieldConstants.isCenterNote(targetNote)) {
            throw new IllegalArgumentException("target note param must be a center note: C1 C2 C3 C4 C5");
        }

        // return paths are for center notes only
        setReturnPath(targetNote);

        // Use note "monitoring" with note camera for center notes only
        addCommands(
                new InstantCommand(() -> s_Shooter.setShooterState(ShooterState.RUNNING)),
                new IntakeAndDeployPivot(s_Pivot, s_Intake, null),
                new DeferredCommand(() -> s_Swerve.makePathFollowingCommand(getInitialPath()), Set.of(s_Swerve))
                        .andThen(new WaitCommand(2)),
                // .deadlineWith(
                // new MonitorForNote(noteVision, () -> s_Swerve.getEstimatedPose(),
                // m_targetNote, this)),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> s_Intake.setIntakeStatus(IntakeStatus.IN_WITH_BEAM_BREAK)),
                                new WaitCommand(0.3)),
                        new WaitCommand(0),
                        () -> s_Intake.getBeamBreak()),
                new ConditionalCommand(
                        new WaitCommand(0),
                        new SequentialCommandGroup
                        (
                                new InstantCommand(() -> s_Shooter.setShooterPosition(ShootPosition.SPEAKER)),
                                s_Swerve.makePathFollowingCommand(m_returnPath),
                                new aimToSpeaker(s_Swerve, null, null, null, s_Pivot, s_Shooter),
                                new AutoPivotShootSequence(s_Pivot, s_Intake, s_Shooter, s_Swerve)
                        ),
                        () -> s_Intake.getBeamBreak()));
    }

    private PathPlannerPath getInitialPath() {
        Pose2d pose = s_Swerve.getPose();
        Pose2d poseBlue = FieldConstants.flipPose(pose);
        // System.out.println("Starting getInitialPath " + poseBlue);

        // this part is used when in center note area, if intended center note is not
        // found
        if (poseBlue.getX() > FieldConstants.BLUE_WING_LINE_X_METERS) {
            Rotation2d heading = m_targetNote.minus(poseBlue.getTranslation()).getAngle();

            // heading here is the heading along the path
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                    new Pose2d(poseBlue.getTranslation(), heading),
                    new Pose2d(m_targetNote, heading));

            // Create the path using the bezier points created above
            // Note final Robot heading should be "backward" since the intake is on the back
            return new PathPlannerPath(
                    bezierPoints,
                    new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                            Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                            Constants.AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared),
                    new GoalEndState(0, heading.rotateBy(Rotation2d.fromRadians(Math.PI)), true));
        }

        Pose2d closestPathStart = poseBlue.nearest(new ArrayList<>(m_candidateStartPaths.keySet()));
        // System.out.println("Center: getInitialPath nearest = " + closestPathStart);
        return m_candidateStartPaths.get(closestPathStart);
    }
}