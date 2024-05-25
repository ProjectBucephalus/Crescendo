package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.IntakeSpit;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.IndexerState;

public class ShootSequence extends SequentialCommandGroup
{
    
    Shooter s_Shooter;
    Intake s_Intake;
    Swerve s_Swerve;
    
    public ShootSequence(Shooter s_Shooter, Intake s_Intake, Swerve s_Swerve) 
    {
        this.s_Shooter = s_Shooter;
        this.s_Intake = s_Intake;
        this.s_Swerve = s_Swerve;
        addCommands
        (
            new ShootSequenceBasic(s_Shooter, s_Intake)
        );
    }
}
