package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.commands.Shooter.AutoPivotShootSequence;
import frc.robot.commands.Shooter.ShootSequence;
import frc.robot.commands.Shooter.ShootSequenceBasic;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteVision;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.PivotPosition;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Swerve;

public class GetMulitNote extends SequentialCommandGroup {

    public GetMulitNote(Translation2d[] noteLocations, Swerve s_Swerve, NoteVision noteVision,
            Shooter s_Shooter, Pivot s_Pivot, Intake s_Intake, Climber s_Climber) {
        // for (Translation2d note : noteLocations) {
        //     if (!FieldConstants.DUMMY_NOTE_WAIT_FLAG.equals(note)) {
        //         addCommands(new AutoPivotShootSequence(s_Pivot, s_Intake, s_Shooter, s_Swerve));
        //     }
        // }
        // Shoot the preloaded note.
        
        addCommands(new WaitCommand(1),
        
        // If not done already
            new InstantCommand(() -> s_Shooter.setShooterState(ShooterState.RUNNING)),
            new InstantCommand(() -> s_Pivot.setPosition(PivotPosition.SPEAKER)),
            // this finishes when no note is in intake (beam break) or after SHOOT_TIME in the command.
            new WaitCommand(0.8),
            new ShootSequenceBasic(s_Shooter, s_Intake),
            new InstantCommand(() -> s_Shooter.setShooterState(ShooterState.RUNNING)),
            new InstantCommand(() -> s_Pivot.setPosition(PivotPosition.DEPLOYED)),
            new WaitCommand(0.2));

        // add all the fetching+shooting NOTE blocks
        for (Translation2d note : noteLocations) {
            if (FieldConstants.DUMMY_NOTE_WAIT_FLAG.equals(note) || noteLocations.length == 0)
            {
                addCommands(new WaitCommand(7));
            }
            else if (FieldConstants.DUMMY_NOTE_GOTOMID_FLAG.equals(note)) 
            {          
                addCommands(new InstantCommand(() -> s_Pivot.setPosition(PivotPosition.STOWED)), new WaitCommand(5), 
                new DeferredCommand(() -> s_Swerve.makePathFollowingCommand(PathPlannerPath.fromPathFile("GoToMid")), Set.of(s_Swerve)));
            } 
            else if (FieldConstants.isCenterNote(note)) 
            {
                addCommands(new GetCentreNote(note, s_Swerve, noteVision, s_Shooter, s_Pivot, s_Intake));
            } 
            else 
            {
                addCommands(new GetStageNote(note, s_Swerve, noteVision, s_Shooter, s_Pivot, s_Intake));
            }
        }
    }
}
