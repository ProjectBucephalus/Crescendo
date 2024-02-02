package frc.robot.commands.Intake.Flap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
<<<<<<< HEAD
import frc.robot.subsystems.Intake.FlapPosition;
/**@author Sebastian Aiello  */
=======

/**
 * intake close flap command
 * @author 5985
 */
>>>>>>> d20d08f27f8a08703a3f0b7545a47dd3614f28a9
public class CloseFlap extends Command {

    Intake s_Intake;

    public CloseFlap (Intake s_Intake) 
    {
        this.s_Intake = s_Intake;
    }
<<<<<<< HEAD

    public void initialize() 
    { 

    }
=======
    public void initialize() 
    {}
>>>>>>> d20d08f27f8a08703a3f0b7545a47dd3614f28a9
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
<<<<<<< HEAD
        s_Intake.setFlapPosition(FlapPosition.CLOSED);
        
    }
    
    public boolean isFinished()
    {
        return true;
=======
        s_Intake.setFlapSpeed(1);     
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) 
    {
        s_Intake.setFlapSpeed(0);
>>>>>>> d20d08f27f8a08703a3f0b7545a47dd3614f28a9
    }
}
