package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.StabiliserPos;

/**
 * PointToAngle Command
 * @author Alec
 */
public class StabiliserBar extends Command 
{
    Intake sIntake;
    StabiliserPos pos;
    public StabiliserBar(Intake sIntake, StabiliserPos pos)
    {
        this.sIntake =sIntake;
        this.pos = pos;
    }

    @Override
    public void execute() 
    {
        //sIntake.setStabliserPos(pos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
