package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.commands.Shooter.AutoPivotShootSequence;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteVision;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class GetMulitNote extends SequentialCommandGroup {

    public GetMulitNote(Translation2d[] noteLocations, Swerve s_Swerve, NoteVision noteVision,
            Shooter s_Shooter, Pivot s_Pivot, Intake s_Intake, Climber s_Climber) {

        // Shoot the preloaded note.
        addCommands(new AutoPivotShootSequence(s_Pivot, s_Intake, s_Shooter, s_Swerve));
        

        // add all the fetching+shooting NOTE blocks
        for (Translation2d note : noteLocations) {
            if (FieldConstants.DUMMY_NOTE_WAIT_FLAG.equals(note) || noteLocations.length == 0) {
                addCommands(new WaitCommand(20));
            } else if (FieldConstants.isCenterNote(note)) {
                addCommands(new GetCentreNote(note, s_Swerve, noteVision, s_Shooter, s_Pivot, s_Intake));
            } else {
                addCommands(new GetStageNote(note, s_Swerve, noteVision, s_Shooter, s_Pivot, s_Intake));
            }
        }
    }
}
