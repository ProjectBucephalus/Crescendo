package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.CAN;
import frc.robot.subsystems.Intake;

public class IntakeSuck extends Command
{
    private final Intake s_Intake;

    public IntakeSuck(Intake s_Intake) 
    {
        this.s_Intake = s_Intake;
        addRequirements(s_Intake);
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        Intake.intakeIn();  
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) 
    {
        Intake.intakeStop();
    }
}
