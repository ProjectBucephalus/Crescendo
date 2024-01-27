package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakePosition;

public class MoveIntakeToPosition extends Command {
    public boolean isFinished = false;

    private Intake s_Intake;
    private IntakePosition position;

    public MoveIntakeToPosition(Intake s_Intake, IntakePosition position) {
        this.s_Intake = s_Intake;
        this.position = position;
    }

    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Intake.setPosition(position);
    }

    public boolean isFinished() {
        return true;
    }
}
