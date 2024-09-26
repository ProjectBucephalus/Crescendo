package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberPosition;

/**
 * Climber retraction command
 * 
 * @author 5985
 */
public class ClimberRetract extends Command 
{
    public Climber s_Climber;

    public ClimberRetract(Climber s_Climber) 
    {
        this.s_Climber = s_Climber;
        addRequirements(s_Climber);
    }

    public void execute() 
    {
        s_Climber.setClimberPosition(ClimberPosition.DOWN);
    }

    public boolean isFinished() 
    {
        return true;
    }
}