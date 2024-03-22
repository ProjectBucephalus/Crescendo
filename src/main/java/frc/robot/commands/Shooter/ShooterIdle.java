package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

/**
 * Shooter idle command
 * @author 5985
 */
public class ShooterIdle extends Command {
    private final Shooter s_Shooter;

    public ShooterIdle(Shooter s_Shooter) 
    {
        this.s_Shooter = s_Shooter;
    }

    // Sets shooterState to the variable IDLE
    public void execute() {
        s_Shooter.setShooterState(ShooterState.IDLE);
    }

    public boolean isFinished() {
        return true;
    }
}
