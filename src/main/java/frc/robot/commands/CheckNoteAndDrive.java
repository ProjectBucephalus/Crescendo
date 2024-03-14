// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.HashMap;
// import java.util.List;
// import java.util.Map;

// import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;

// import frc.robot.FieldConstants;
// import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.NoteVision;

// //this is only for the center 5 notes 
// //we can make a different command if we want to generate a trajectory to the note on the fly 
// public class CheckNoteAndDrive extends Command {
//     /** Creates a new checkNoteAndDrive. */
//     private Swerve s_Swerve;
//     private NoteVision m_noteVision;

//     private Command m_followTrajectory;

//     private int m_wantedNote;
//     private int m_backUpNote;

//     private static final Map<Integer, Pose2d> NOTE_POSITIONS = new HashMap<Integer, Pose2d>() {
//         {
//             put(1, FieldConstants.NOTE_C_1);
//             put(2, FieldConstants.NOTE_C_2);
//             put(3, FieldConstants.NOTE_C_3);
//             put(4, FieldConstants.NOTE_C_4);
//             put(5, FieldConstants.NOTE_C_5);
//         }
//     };

//     private static final Map<Integer, PathPlannerPath> ROBOT_PATHS = new HashMap<Integer, PathPlannerPath>() {
//         {
//             put(1, PathPlannerPath.fromPathFile("Test"));
//             put(2, PathPlannerPath.fromPathFile("Test"));
//             put(3, PathPlannerPath.fromPathFile("Test"));
//             put(4, PathPlannerPath.fromPathFile("Test"));
//             put(5, PathPlannerPath.fromPathFile("Test"));
//         }
//     };

//     public CheckNoteAndDrive(Swerve s_Swerve, NoteVision noteVision, int wantedNote, int backUpNote) {
//         // Use addRequirements() here to declare subsystem dependencies.
//         s_Swerve = s_Swerve;
//         m_noteVision = noteVision;
//         m_backUpNote = backUpNote;
//         m_wantedNote = wantedNote;
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
//         m_followTrajectory = null;

//         // field relative note positions
//         List<Pose2d> notes = m_noteVision.getNotes(s_Swerve.getEstimatedPose());

//         Translation2d wantedNoteTranslation = NOTE_POSITIONS.get(m_wantedNote).getTranslation();
//         for (Pose2d note : notes) {
//             // goes through the notes in the note list and checks if the wanted note pose is
//             // in there .2 is arbitrary and will need to be tuned for accuracy
//             if (note.getTranslation().getDistance(wantedNoteTranslation) <= 0.2) {
//                 m_followTrajectory = s_Swerve.makePathFollowingCommand(ROBOT_PATHS.get(m_wantedNote));
//                 break;
//             }

//         }
//         if (m_followTrajectory == null)
//             m_followTrajectory = s_Swerve.makePathFollowingCommand(ROBOT_PATHS.get(m_backUpNote));

//         m_followTrajectory.initialize();
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//         if (m_followTrajectory != null)
//             m_followTrajectory.execute();
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
//         // if interrupted, stop the follow trajectory
//         DriverStation.reportError(String.format("NoteCheckAndDrive end interrupted = %s", interrupted), false);

//         if (m_followTrajectory != null)
//             m_followTrajectory.end(interrupted);
//         m_followTrajectory = null;
//     }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         return m_followTrajectory == null || m_followTrajectory.isFinished();
//     }
// }