package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.PivotPosition;

public class MoveIntakeToPosition extends Command {
    public boolean isFinished = false;

    private Pivot s_Pivot;
    private PivotPosition position;

    public MoveIntakeToPosition(Pivot s_Pivot, PivotPosition position) {
        this.s_Pivot = s_Pivot;
        this.position = position;
    }

    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Pivot.setPosition(position);
    }

    public boolean isFinished() {
        return true;
    }
}
