package frc.robot.commands.BuddyClimb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.BuddyClimbPosition;

public class StopBuddyClimber extends Command {
    private Climber s_Climber;

    public StopBuddyClimber(Climber s_Climber) {
        this.s_Climber = s_Climber;
    }

    @Override
    public void execute() {
        s_Climber.setBuddyClimb(BuddyClimbPosition.STOPPED);
    }

    public boolean isFinished() {
        return true;
    }

    
}
