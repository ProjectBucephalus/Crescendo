package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.IntakeSpit;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.IndexerState;
import frc.robot.subsystems.Intake.IntakeStatus;

public class PullNoteSequence extends SequentialCommandGroup{
    
    Intake s_Intake;
    
    public PullNoteSequence(Intake s_Intake) 
    {
        this.s_Intake = s_Intake;
        addCommands
        (   
            new InstantCommand(() -> s_Intake.setIndexerState(IndexerState.STOPPED)),
            new WaitCommand(0.2),
            new IntakeSuck(s_Intake),
            new WaitCommand(0.05),
            new IntakeStop(s_Intake)
        );
    }
}