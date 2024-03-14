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

    public void initialize() {
    }

    public void execute() 
    {
        s_Climber.setClimberPosition(ClimberPosition.DOWN);
        // // Retracts until below minimum climber position
        // if (s_Climber.getPosition() > Constants.Climber.climberDownPos)
        // {
        // s_Climber.setSpeed(-1);
        // // System.out.println(s_Climber.getPosition());
        // // System.out.println("Running");
        // }
        // else
        // {
        // s_Climber.setSpeed(0);
        // isFinished = true;
        // }
    }

    public void end() {
    }

    public boolean isFinished() {
        return true;
    }
}