package frc.robot.commands;

import java.util.ArrayList;
import java.util.Set;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.commands.Intake.GetBeamBreak;
import frc.robot.commands.Intake.IntakeAndDeployPivot;
import frc.robot.commands.Shooter.AutoPivotShootSequence;
import frc.robot.commands.Shooter.ShootSequence;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShootPosition;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Pivot;

// Note this is a SequentialCommandGroup
public class GetStageNote extends GetNote {

    public GetStageNote(Translation2d targetNote, Swerve s_Swerve, NoteVision noteVision,
            Shooter s_Shooter, Pivot s_Pivot, Intake s_Intake) {
        super(targetNote, s_Swerve, noteVision, s_Shooter, s_Intake);

        if (FieldConstants.isCenterNote(targetNote)) {
            throw new IllegalArgumentException("target note param must be a stage note: S1 S2 S3");
        }

        addCommands(
                new InstantCommand(() -> s_Shooter.setShooterState(ShooterState.RUNNING)),
                new IntakeAndDeployPivot(s_Pivot, s_Intake, null),
                new DeferredCommand(() -> s_Swerve.makePathFollowingCommand(getInitialPath()), Set.of(s_Swerve)),
                new ParallelDeadlineGroup(
                        new GetBeamBreak(s_Intake),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> s_Shooter.setShooterPosition(ShootPosition.SPEAKER)),
                                new WaitCommand(0.1),
                                new AutoPivotShootSequence(s_Pivot, s_Intake, s_Shooter))),
                new ShootSequence(s_Shooter, s_Intake));
    }

    private PathPlannerPath getInitialPath() {
        Pose2d pose = s_Swerve.getEstimatedPose();
        Pose2d poseBlue = FieldConstants.flipPose(pose);
        System.out.println("Starting getInitialPath " + poseBlue);

        Pose2d closestPathStart = poseBlue.nearest(new ArrayList<>(m_candidateStartPaths.keySet()));
        System.out.println("getInitialPath nearest = " + closestPathStart);
        return m_candidateStartPaths.get(closestPathStart);
    }
}