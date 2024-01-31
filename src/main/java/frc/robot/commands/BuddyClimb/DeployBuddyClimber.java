package frc.robot.commands.BuddyClimb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.BuddyClimbPosition;

public class DeployBuddyClimber extends Command {
    private Intake s_Intake;

    public DeployBuddyClimber(Intake s_Intake) {
        this.s_Intake = s_Intake;
    }

    @Override
    public void execute() {
        s_Intake.setBuddyClimb(BuddyClimbPosition.RUNNING);
    }

    
    
}
