package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeStatus;

/**
 * Intake spit command
 * @author 5985
 */
public class IntakeSpit extends Command {
    private final Intake s_Intake;

    public IntakeSpit(Intake s_Intake) {
        this.s_Intake = s_Intake;
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Intake.setIntakeStatus(IntakeStatus.OUT);
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
