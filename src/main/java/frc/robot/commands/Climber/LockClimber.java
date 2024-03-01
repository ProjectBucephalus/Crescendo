package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/**
* Locks the climber from moving up or down
* @author 5985
*/

public class LockClimber extends Command {

    Climber s_Climber;
    public LockClimber(Climber s_Climber) {
        this.s_Climber = s_Climber;
    }

    private void setClimberState() {

    }

    public boolean isFinished() {
        return true;
    }
}
