package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IndexerPosition;
import frc.robot.subsystems.Intake.IndexerPosition;
import frc.robot.subsystems.Intake.IntakeStatus;

/**
 * intake suck command
 * @author 5985
 */
public class IntakeSuck extends Command {

    Intake s_Intake;

    public IntakeSuck(Intake s_Intake) {
        this.s_Intake = s_Intake;
    }

    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        s_Intake.setIntakeStatus(IntakeStatus.IN_WITH_BEAM_BREAK);

    }

    // Called once the command ends or is interrupted.
    @Override
    public boolean isFinished() {
        return true;
    }
}
