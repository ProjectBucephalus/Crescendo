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

public class PushNoteSequence extends SequentialCommandGroup{
    
    Intake s_Intake;
    
    public PushNoteSequence(Intake s_Intake) 
    {
        this.s_Intake = s_Intake;
        addCommands
        (
            new IntakeSpit(s_Intake),
            new WaitCommand(0.035),
            new IntakeStop(s_Intake),
            new InstantCommand(() -> s_Intake.setIndexerState(IndexerState.IN_FOR_SHOOTING))
        );
    }
}
