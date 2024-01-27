package frc.robot.commands.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

/**
 * shooting sequence command
 * @author 5985
 */
public class ShootSequence extends Command {
    private final Intake s_Intake;
    private static TalonFX mBottomShooter = new TalonFX(Constants.Shooter.mBottomShooterID);
    private static TalonFX mTopShooter = new TalonFX(Constants.Shooter.mTopShooterID);

    /**
     * shooting sequence
     * @param s_Intake a reference to the intake subsystem
     */
    public ShootSequence(Intake s_Intake) {
        this.s_Intake = s_Intake;
        addRequirements(s_Intake);
    }

    public void execute() {
        double mShooterSpeedDecimal = (Math.abs(mTopShooter.getRotorVelocity().getValueAsDouble()) / 512); // Output absolute motor speed [0..1]
        s_Intake.spinShooter();
        if ((mShooterSpeedDecimal > Math.abs(mTopShooter.get() * 0.9))) {
            // s_Intake.openFlap();
            s_Intake.setIntakeSpeed(-1);
        }
    }

    public void end(boolean interrupted) {
        // s_Intake.closeFlap();
        s_Intake.idleShooter();
        s_Intake.setIntakeSpeed(0);
    }
}
