package frc.robot.commands.BuddyClimb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.BuddyClimbPosition;

/**
 * Deploy buddy climber command to start the spool running
 * @author 5985
 */
public class DeployBuddyClimber extends Command {
    private Climber s_Climber;

    public DeployBuddyClimber(Climber s_Climber) {
        this.s_Climber = s_Climber;
    }

    /**
     * Sets the buddyClimbPosition to the variable RUNNING
     */
    @Override
    public void execute() {
        s_Climber.setBuddyClimb(BuddyClimbPosition.RUNNING);
    }

    public boolean isFinished() {
        return true;
    }

    
    
}
