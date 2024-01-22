package frc.robot.commands.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ShootSequence extends Command {
    private final Intake s_Intake;
    // motors
    private static TalonFX mShooter = new TalonFX(Constants.mShooterID);

    /* sets subsystem requirements */
    public ShootSequence(Intake s_Intake) 
    {
        this.s_Intake = s_Intake;
        addRequirements(s_Intake);
    }

    /* spins up the shooter, then opens the flap and fires once it's up to speed */
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

    /* idles shooter, and closes flap */
    public void end(boolean interrupted) 
    {
        Intake.closeFlap();
        Intake.idleShooter();
        Intake.intakeStop();
    } 
}
