package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

public class ShooterRev extends Command {
    private final Shooter s_Shooter;

    public ShooterRev(Shooter s_Shooter) 
    {
        this.s_Shooter = s_Shooter;
    }

    public void execute() {
        s_Shooter.setShooterState(ShooterState.RUNNING);
    }

    public boolean isFinished() {
        return true;
    }
}
