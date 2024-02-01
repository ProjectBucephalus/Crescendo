package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

/**
 * climber extension command
 * @author 5985
 */
public class ClimberExtend extends Command{

    public Climber s_Climber;
    public boolean isFinished;

    public ClimberExtend(Climber s_Climber) 
    {
        this.s_Climber = s_Climber;
        addRequirements(s_Climber);
        isFinished = false;
    }

    public void initialize() 
    {}


    public void execute() 
    {
        s_Climber.setSpeed(1);
        // Climbs until above max climber position
        if (s_Climber.getPosition() < (360 * (Constants.Climber.maxExtensionSpoolRotations * Constants.Climber.motorToSpoolGearRatio))) // Climber spool rotation * climber motor gear ratio, converted to degrees
        { 
            s_Climber.setSpeed(1);
        } else {
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
