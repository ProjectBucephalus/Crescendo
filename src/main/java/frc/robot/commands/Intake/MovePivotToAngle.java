package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.PivotPosition;

/**
 * Move to intake position command
 * @author 5985
 */
public class MovePivotToAngle extends Command {
    public boolean isFinished = true;
    private Pivot s_Pivot;
    private double angle;
    

    public MovePivotToAngle(Pivot s_Pivot, double angle) {
        this.s_Pivot = s_Pivot;
        this.angle = angle;
    }

    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Pivot.setDesiredPostion(angle);
    }

    public boolean isFinished() {
        return isFinished;
    }
}
