package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pivot.PivotPosition;
import frc.robot.subsystems.Shooter.ShooterState;

/**
 * This moves the pivot and shoots into the speaker.
 */
public class AutoPivotShootSequence extends SequentialCommandGroup{
    
    public AutoPivotShootSequence(Pivot s_Pivot, Intake s_Intake, Shooter s_Shooter) {
        addCommands(
            // If not done already
            new InstantCommand(() -> s_Shooter.setShooterState(ShooterState.RUNNING)),
            new InstantCommand(() -> s_Pivot.setPosition(PivotPosition.SPEAKER)),
            // this finishes when no note is in intake (beam break) or after SHOOT_TIME in the command.
            new WaitCommand(0.2),
            new ShootSequence(s_Shooter, s_Intake),
            new InstantCommand(() -> s_Shooter.setShooterState(ShooterState.RUNNING)),
            new InstantCommand(() -> s_Pivot.setPosition(PivotPosition.DEPLOYED)),
            new WaitCommand(0.2)
        );
    }


}
