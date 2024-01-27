package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/**
 * climber extension command
 * @author 5985
 */
public class ClimberExtend extends Command{

    public Climber s_Climber;
    public boolean isFinished;

    public ClimberExtend(Climber s_Climber) {
        this.s_Climber = s_Climber;
        isFinished = false;
    }

    public void initialize() 
    {}


    public void execute() 
    {
        s_Climber.setSpeed(1);
        // Climbs until above max climber position
        if (s_Climber.getPosition() < 10000000) { //TODO Put the corect number in for max climb pos
            s_Climber.setSpeed(1);
        }
        else
        {
            s_Climber.setSpeed(0);
            isFinished = true;
        }
    }

    public boolean isFinished() 
    {
        return isFinished;
            
    }
}
