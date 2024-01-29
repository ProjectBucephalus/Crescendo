package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class IntakeSpit extends Command {
    private final Intake s_Intake;

    public IntakeSpit(Intake s_Intake) {
        this.s_Intake = s_Intake;
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Intake.setIntakeSpeed(0.70);
    }

    public void end(boolean interrupted) {
        s_Intake.setIntakeSpeed(0);
    }
}
