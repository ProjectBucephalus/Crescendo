package frc.robot.commands;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.FieldConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteVision;
import frc.robot.subsystems.Shooter;

// This abstract class represents a command group for getting notes during autonomous mode.
// It defines a set of paths to navigate to different note locations on the field.
public abstract class GetNote extends SequentialCommandGroup {

    
    protected static final Map<Translation2d, String[]> s_pathLookup = new HashMap<>() {
        {
            put(FieldConstants.NOTE_C_1, new String[] { "Start_1 to Note_C_1", "Start_2 to Note_C_1", "Note_C_1 to Shoot_1" });
            put(FieldConstants.NOTE_C_2, new String[] { "Start_1 to Note_C_2", "Start_2 to Note_C_2", "Shoot_1 to Note_C_2", "Note_C_2 to Shoot_1" });
            put(FieldConstants.NOTE_C_3, new String[] { "Start_2 to Note_C_3", "Note_C_3 to Shoot_3" });
            put(FieldConstants.NOTE_C_4, new String[] { "Start_3 to Note_C_4", "Note_C_4 to Shoot_2" });
            put(FieldConstants.NOTE_C_5, new String[] { "Start_3 to Note_C_5", "Note_C_5 to Shoot_2" });

            put(FieldConstants.BLUE_NOTE_S_1, new String[] { "Start_1 to Note_S_1", "Start_2 to Note_S_1", "Start_3 to Note_S_1", "Note_S_2 to Note_S_1", null });
            put(FieldConstants.BLUE_NOTE_S_2, new String[] { "Start_1 to Note_S_2", "Start_2 to Note_S_2", "Start_3 to Note_S_2", "Note_S_1 to Note_S_2", "Note_S_3 to Note_S_2", null });
            put(FieldConstants.BLUE_NOTE_S_3, new String[] { "Start_1 to Note_S_3", "Start_2 to Note_S_3", "Start_3 to Note_S_3", "Note_S_2 to Note_S_3", null });
            
        }
    };

    protected final Map<Pose2d,PathPlannerPath> m_candidateStartPaths = new LinkedHashMap<>();
    protected final Swerve s_Swerve;
    protected final Translation2d m_targetNote; 

    private void initPaths(String[] pathnameArray) {
        for(int i=0; i<pathnameArray.length-1; i++) {
            // Load each path from file and add its starting pose to the candidate start paths.
            PathPlannerPath path = FieldConstants.loadPath(pathnameArray[i]);
            if (path != null) {
                Pose2d startPose = path.getStartingDifferentialPose();
                m_candidateStartPaths.put(startPose, path);
            }
        }
    }

    public GetNote(Translation2d targetNote, Swerve s_Swerve, NoteVision noteVision, Shooter shooter, Intake intake) {
        this.m_targetNote = targetNote;
        this.s_Swerve = s_Swerve;
        initPaths(s_pathLookup.get(targetNote));

        // addCommands(new PrintCommand("**** GetNoteX-- Starting Auto target note: "+ targetNote));
    }
}