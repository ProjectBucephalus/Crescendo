package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakePosition;

/**
 * intake stow command
 * @author 5985
 */
public class IntakeStow extends Command {

    Intake s_Intake;
    public boolean isFinished = false;

    public IntakeStow(Intake s_Intake) {
        this.s_Intake = s_Intake;
    }

    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Intake.setPosition(IntakePosition.STOWED);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    public boolean isFinished() {
        return true;
    }
}
