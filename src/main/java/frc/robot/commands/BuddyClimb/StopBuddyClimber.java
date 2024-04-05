package frc.robot.commands.BuddyClimb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.BuddyClimbPosition;

/**
 * Stop buddy climber command
 * @author 5985
 */
public class StopBuddyClimber extends Command {
    private Climber s_Climber;

    public StopBuddyClimber(Climber s_Climber) {
        this.s_Climber = s_Climber;
    }

    @Override
    public void execute() {
        // Sets the buddyClimberPosition to the variable STOPPED
        s_Climber.setBuddyClimb(BuddyClimbPosition.STOPPED);
    }

    public boolean isFinished() {
        return true;
    }

    
}
