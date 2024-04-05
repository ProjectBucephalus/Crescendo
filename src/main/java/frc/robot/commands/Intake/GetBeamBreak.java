package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class GetBeamBreak extends Command {
    private Intake s_Intake;

    public GetBeamBreak(Intake s_Intake) {
        this.s_Intake = s_Intake;
    }

    @Override
    public boolean isFinished() {
        return s_Intake.getBeamBreak();
    }
}
