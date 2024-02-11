package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeStatus;

/**
 * intake split command
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

    /**
     * stops the intake
     * @param interrupted
     */
    public void end(boolean interrupted) {
        s_Intake.setIntakeStatus(IntakeStatus.STOPPED);
    }
}
