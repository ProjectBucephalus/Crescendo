package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

public class ShooterIdle extends Command {
    private final Shooter s_Shooter;

    public ShooterIdle(Shooter s_Shooter) 
    {
        this.s_Shooter = s_Shooter;
    }

    public void execute() {
        s_Shooter.setShooterState(ShooterState.IDLE);
    }

    public boolean isFinished() {
        return true;
    }
}
