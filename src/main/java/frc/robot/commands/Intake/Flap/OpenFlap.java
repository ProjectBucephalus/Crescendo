package frc.robot.commands.Intake.Flap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.FlapPosition;

<<<<<<< HEAD
/**@author Sebastian Aiello  */
=======
/**
 * intake open flap command
 * @author 5985
 */
>>>>>>> d20d08f27f8a08703a3f0b7545a47dd3614f28a9
public class OpenFlap extends Command {

    Intake s_Intake;

    public OpenFlap (Intake s_Intake) 
    {
        this.s_Intake = s_Intake;
    }

    public void initialize() 
    { 

    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        s_Intake.setFlapPosition(FlapPosition.OPEN);
        
    }
    
    public boolean isFinished() 
    {
        return true;
    }

}
