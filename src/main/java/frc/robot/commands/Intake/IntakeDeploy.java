package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakePosition;

public class IntakeDeploy extends Command {
    public boolean isFinished = false;
    Intake s_Intake;

    public IntakeDeploy(Intake s_Intake) {
        this.s_Intake = s_Intake;
    }

    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Intake.setPosition(IntakePosition.DEPLOYED);
    }

    public boolean isFinished() {
        return true;
    }
}
