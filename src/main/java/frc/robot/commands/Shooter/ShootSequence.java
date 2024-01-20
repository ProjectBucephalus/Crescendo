package frc.robot.commands.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ShootSequence extends Command {
    private final Intake s_Intake;
    private static TalonFX mShooter = new TalonFX(Constants.mShooterID);

    public ShootSequence(Intake s_Intake) 
    {
        this.s_Intake = s_Intake;
        addRequirements(s_Intake);
    }

    public void execute() 
    {
        Intake.spinShooter();
        Intake.openFlap();
        while (mShooter.getRotorVelocity() ) {
            
        }
    }

    public void end(boolean interrupted) 
    {
        Intake.closeFlap();
        Intake.idleShooter();
    } 
}
