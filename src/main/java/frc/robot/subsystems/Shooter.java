package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
        OUT,
        TRAP
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

    public Shooter() 
    {
        SmartDashboard.putNumber("Shooter Bottom Speed", 0);
        SmartDashboard.putNumber("Shooter Top Speed", 0);
    }

    /**
     * Sets the shooter's state based on an enum
     * 
     * @param state Enum representing the desired status of the shooter
     * @author 5985
     */
    public void setShooterState(ShooterState state) 
    {
        SmartDashboard.putString("Current State of Shooter Motors for sim", state.name());

        switch (state) 
        {
            case RUNNING:
                driveDutyCycle.Output = bottomSpeed;
                mBottomShooter.setControl(driveDutyCycle);

                driveDutyCycle.Output = topSpeed;
                mTopShooter.setControl(driveDutyCycle);
                break;
            case STOPPED:
                driveDutyCycle.Output = 0;
                mBottomShooter.setControl(driveDutyCycle);

                driveDutyCycle.Output = 0;
                mTopShooter.setControl(driveDutyCycle);
            case IDLE:
                //System.out.println("idle");
                driveDutyCycle.Output = Constants.Shooter.shooterIdleSpeed;
                mBottomShooter.setControl(driveDutyCycle);

                driveDutyCycle.Output = Constants.Shooter.shooterIdleSpeed;
                mTopShooter.setControl(driveDutyCycle);
                break;
            case OUT:
                driveDutyCycle.Output = Constants.Shooter.shooterEjectSpeed;
                mBottomShooter.setControl(driveDutyCycle);

                driveDutyCycle.Output = Constants.Shooter.shooterEjectSpeed;
                mTopShooter.setControl(driveDutyCycle);
            case TRAP:
                driveDutyCycle.Output = SmartDashboard.getNumber("Shooter Bottom Speed", 0);
                mBottomShooter.setControl(driveDutyCycle);

                driveDutyCycle.Output = SmartDashboard.getNumber("Shooter Top Speed", 0);
                mTopShooter.setControl(driveDutyCycle);
                break;
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
     * @return Boolean, true when current shooter RPM is acceptable
     * @author 5985
     * @author Aidan
     */
    public boolean rpmWithinTolerance() {
        return true; // TODO
    }

    @Override
    public void periodic() {
        // Prints info to Smart Dashboard
        SmartDashboard.putString("Where am I shooting", getShootPosition().name());
        SmartDashboard.putString("Current State of Motors for sim", getShootPosition().name());
    }

    
}