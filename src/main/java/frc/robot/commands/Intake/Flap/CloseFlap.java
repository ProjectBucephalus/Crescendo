package frc.robot.commands.Intake.Flap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class CloseFlap extends Command {

    frc.robot.subsystems.Intake s_Intake;

    public CloseFlap (Intake s_Intake) {
        this.s_Intake = s_Intake;
    }
    public void initialize() {
       
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    
        s_Intake.setFlapSpeed(1);
        
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_Intake.setFlapSpeed(0);
    
    }
}
