package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeOut extends Command
{

    Intake s_Intake;

    public IntakeOut(Intake s_Intake) {
        this.s_Intake = s_Intake;
    }

    public void initialize() {
       
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    
    s_Intake.intakeOut();  
        
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
       
    s_Intake.intakeStop();
    } 
}
