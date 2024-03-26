package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IDConstants;

public class Shooter extends SubsystemBase {
    // motors
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);

    public TalonFX mTopShooter = new TalonFX(IDConstants.Intooter.Shooter.mTopShooterID);
    public TalonFX mBottomShooter = new TalonFX(IDConstants.Intooter.Shooter.mBottomShooterID);

    // Sets the starting shooter aiming position to the speaker
    private ShootPosition shooterMode = ShootPosition.SPEAKER;

    /**
     * Enum representing the status of the indexer roller (OPEN for running, CLOSED
     * for stopped)
     * 
     * @author 5985
     */
    public enum FlapPosition {
        OPEN,
        CLOSED,
    };

    /**
     * Enum representing the status of the shooter
     * 
     * @author 5985
     */
    public enum ShooterState {
        RUNNING,
        STOPPED,
        IDLE,
        OUT
    };

    /**
     * Enum representing the shooter's aiming position
     * 
     * @author 5985
     */
    public enum ShootPosition {
        AMP,
        SPEAKER,
        TRAP,
    };

    public Shooter() {
    }

    /**
     * Sets the shooter's state based on an enum
     * 
     * @param state Enum representing the desired status of the shooter
     * @author 5985
     */
    public void setShooterState(ShooterState state) {
        SmartDashboard.putString("Current State of Shooter Motors for sim", state.name());

        switch (state) {
            case RUNNING:
                driveDutyCycle.Output = Constants.Shooter.runningBottomShooterSpeed;
                mBottomShooter.setControl(driveDutyCycle);

                driveDutyCycle.Output = Constants.Shooter.runningTopShooterSpeed;
                mTopShooter.setControl(driveDutyCycle);
                break;
            case STOPPED:
                driveDutyCycle.Output = 0;
                mBottomShooter.setControl(driveDutyCycle);
                mTopShooter.setControl(driveDutyCycle);
                break;
            case IDLE:
                //System.out.println("idle");
                driveDutyCycle.Output = Constants.Shooter.shooterIdleSpeed;
                mBottomShooter.setControl(driveDutyCycle);

                driveDutyCycle.Output = Constants.Shooter.shooterIdleSpeed;
                mTopShooter.setControl(driveDutyCycle);
                break;
            case OUT:
                driveDutyCycle.Output = -0.5;
                mBottomShooter.setControl(driveDutyCycle);

                driveDutyCycle.Output = -0.5;
                mTopShooter.setControl(driveDutyCycle);
            default:
                break;
        }
        // SmartDashboard.putNumber("bottomShooterSpeed", bottomSpeed);
        // SmartDashboard.putNumber("topShooterSpeed", topSpeed);
    }

    public void setShooterPosition(ShootPosition pos) {
        shooterMode = pos;
    }

    public ShootPosition getShootPosition() {
        return shooterMode;
    }

    /**
     * Checks if shooter RPM is within acceptable tolerance.
     * TODO not implimented yet
     * 
     * @return Boolean, true when current shooter RPM is acceptable
     * @author 5985
     * @author Aidan
     */
    public boolean rpmWithinTolerance() {
        return mTopShooter.getVelocity().getValueAsDouble() > Constants.Shooter.ShooterAcceptableVelocity;

    }

    @Override
    public void periodic() {
        // Prints info to Smart Dashboard
        SmartDashboard.putString("Where am I shooting", getShootPosition().name());
        SmartDashboard.putString("Current State of Motors for sim", getShootPosition().name());

        SmartDashboard.putNumber("Top Shooter RPS", mTopShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Bottom Shooter RPS", mBottomShooter.getVelocity().getValueAsDouble());
    }

}