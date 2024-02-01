package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

/**
 * climber retraction command
 * @author 5985
 */
public class ClimberRetract extends Command {

    public Climber s_Climber;
    public boolean isFinished;

    public ClimberRetract(Climber s_Climber) 
    {
        this.s_Climber = s_Climber;
        addRequirements(s_Climber);
        isFinished = false;
    }

    public void initialize() 
    {}

    public void execute() 
    {
        if (s_Climber.getPosition() > Constants.Climber.climberDownPos) 
        {
            s_Climber.setSpeed(-1);
            System.out.println(s_Climber.getPosition());
            System.out.println("Running");
        } 
        else 
        {
            s_Climber.setSpeed(0);
            isFinished = true;
        }
    }

    public void end() 
    {
        s_Climber.setSpeed(0);
    }

    public boolean isFinished() 
    {
        return isFinished;
    }
}