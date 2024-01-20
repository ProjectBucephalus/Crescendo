package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.CAN;
import frc.robot.subsystems.Intake;

public class IntakeStowed extends Command
{
    public void initialize() {
       
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        if (Intake.inLimitSwitch.get()) 
        {
            Intake.intakeArmStop();
        }
        else
        {
            Intake.setIntakeStowed();
        }
    }
   
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) 
    {
    
    
    }
}