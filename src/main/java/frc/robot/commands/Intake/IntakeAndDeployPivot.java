package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Intake.FlapPosition;
import frc.robot.subsystems.Pivot.PivotPosition;

public class IntakeAndDeployPivot extends Command {
    public boolean isFinished = false;
    Pivot s_Pivot;
    Intake s_Intake;

    public IntakeAndDeployPivot(Pivot s_Pivot, Intake s_Intake) {
        this.s_Pivot = s_Pivot;
        this.s_Intake = s_Intake;
    }

    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Pivot.setPosition(PivotPosition.DEPLOYED);
        s_Intake.setIntakeSpeed(-1);
        s_Intake.setFlapPosition(FlapPosition.CLOSED);
    }

    public boolean isFinished() {
        return true;
    }
}
