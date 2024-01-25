package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

import frc.robot.Constants;

public class IntakeDeploy extends Command
{

    Intake s_Intake;

    public IntakeDeploy(Intake s_Intake) {
        this.s_Intake = s_Intake;
        
    }

    
    public void initialize() {
       
    }
    
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Intake.moveArmToAngle(1);
        s_Intake.setIntakeSpeed(-1);  
        SmartDashboard.putNumber("Arm Pos", s_Intake.getArmPos());
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {   
    }
}
