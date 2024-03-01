package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeStatus;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.PivotPosition;

/**
 * Intake stow command
 * @author 5985
 */
public class StopIntakeAndStow extends Command {

    Pivot s_Pivot;
    Intake s_Intake;
    public boolean isFinished = false;

    public StopIntakeAndStow(Pivot s_Pivot, Intake s_Intake) {
        this.s_Pivot = s_Pivot;
        this.s_Intake = s_Intake;
    }

    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Pivot.setPosition(PivotPosition.STOWED);
        s_Intake.setIntakeStatus(IntakeStatus.STOPPED);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return true;
    }
}
