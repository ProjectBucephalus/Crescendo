package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberStatus;

/**
 * Lock Climber command
 * @author 5985
 */
public class LockClimber extends Command {

    Climber s_Climber;

    public LockClimber(Climber s_Climber) {
        this.s_Climber = s_Climber;
    }

    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        s_Climber.setStatus(ClimberStatus.LOCKED);

    }

    // Called once the command ends or is interrupted.
    @Override
    public boolean isFinished() {
        return true;
    }
}
