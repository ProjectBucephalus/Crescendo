package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/**
 * climber retraction command
 * @author 5985
 */
public class ClimberRetract extends Command {

    public Climber s_Climber;
    public boolean isFinished;

    public ClimberRetract(Climber s_Climber) {
        this.s_Climber = s_Climber;
        isFinished = false;
    }

    public void initialize() 
    {}

    public void execute() 
    {
        s_Climber.setSpeed(1);
        
        if (s_Climber.getPosition() == 0) {
            s_Climber.setSpeed(1);
        } else {
            s_Climber.setSpeed(0);
            isFinished = true;
        }

    }

    public boolean isFinished() 
    {
        return isFinished;
    }
}
