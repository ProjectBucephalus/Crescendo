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
        double mShooterSpeedDecimal = (Math.abs(mShooter.getRotorVelocity().getValueAsDouble()) / 512); // Output absolute motor speed [0..1]
        Intake.spinShooter();
        if ((mShooterSpeedDecimal > Math.abs(mShooter.get() * 0.9))) 
        {
            Intake.openFlap();
            Intake.intakeIn();
        }
    }

    public void end(boolean interrupted) 
    {
        Intake.closeFlap();
        Intake.idleShooter();
        Intake.intakeStop();
    } 
}
