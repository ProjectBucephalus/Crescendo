package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.IntakeAndDeployPivot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.IntakeStatus;
import frc.robot.subsystems.Pivot.PivotPosition;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Shooter.ShootPosition;;

/**
 * This moves the pivot and shoots into the speaker.
 */
public class TrapShootSequence extends SequentialCommandGroup{
    
    public TrapShootSequence(Pivot s_Pivot, Intake s_Intake, Shooter s_Shooter, Swerve s_Swerve) {
            addCommands
            (
                new InstantCommand(() -> s_Shooter.setShooterState(ShooterState.TRAP)),
                new InstantCommand(() -> s_Pivot.setPosition(PivotPosition.DEPLOYED)),
                new InstantCommand(() -> s_Shooter.setShooterPosition(ShootPosition.TRAP)),
                new WaitCommand(0.8),
                new ShootSequence(s_Shooter, s_Intake, s_Swerve),
                new WaitCommand(0.2),
                new InstantCommand(() -> s_Shooter.setShooterState(ShooterState.IDLE)),
                new InstantCommand(() -> s_Pivot.setPosition(PivotPosition.STOWED)),
                new InstantCommand(() -> s_Shooter.setShooterPosition(ShootPosition.SPEAKER))
            );
    }


}
