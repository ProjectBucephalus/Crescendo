package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ShooterRev extends Command {
    private final Intake s_Intake;
    

    /* sets subsystem requirements */
    public ShooterRev(Intake s_Intake) 
    {
        this.s_Intake = s_Intake;
        addRequirements(s_Intake);
    }

    /* spins up the shooter to idle speed */
    public void execute() 
    {
        Intake.idleShooter();
    }

    public void end(boolean interrupted) 
    {
        Intake.stopShooter();
    } 
}
