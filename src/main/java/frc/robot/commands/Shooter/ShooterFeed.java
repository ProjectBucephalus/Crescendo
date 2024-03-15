package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeStatus;

/**
 * Intake suck command
 * @author 5985
 */
public class ShooterFeed extends Command {

    Intake s_Intake;

    public ShooterFeed(Intake s_Intake) {
        this.s_Intake = s_Intake;
    }

    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        s_Intake.setIntakeStatus(IntakeStatus.IN_FOR_SHOOTING);

    }

    // Called once the command ends or is interrupted.
    @Override
    public boolean isFinished() {
        return true;
    }
}
