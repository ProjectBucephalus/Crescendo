package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.IDConstants;

public class Shooter extends SubsystemBase 
{
    // motors
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);

    public TalonFX mTopShooter = new TalonFX(IDConstants.Intooter.Shooter.mTopShooterID);
    public TalonFX mBottomShooter = new TalonFX(IDConstants.Intooter.Shooter.mBottomShooterID);

    // Sets the starting shooter aiming position to the speaker
    private ShootPosition shooterMode = ShootPosition.SPEAKER;

    private double idleSpeed = Constants.Shooter.shooterIdleSpeed;

    /**
     * Enum representing the status of the shooter
     * 
     * @author 5985
     */
    public enum ShooterState 
    {
        RUNNING,
        STOPPED,
        IDLE,
        OUT,
        TRAP,
        LOB
    };

    /**
     * Enum representing the shooter's aiming position
     * 
     * @author 5985
     */
    public enum ShootPosition 
    {
        AMP,
        SPEAKER,
        TRAP,
    };

    public Shooter() 
    {
        mBottomShooter.getConfigurator().apply(CTREConfigs.bottomShooterMotorFXConfig);
        mTopShooter.getConfigurator().apply(CTREConfigs.topShooterMotorFXConfig);
    }

    /**
     * Sets the shooter's state based on an enum
     * 
     * @param state Enum representing the desired status of the shooter
     * @author 5985
     */
    public void setShooterState(ShooterState state) 
    {
        switch (state) 
        {
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
                if (SmartDashboard.getBoolean("Test Mode", false))
                {
                    idleSpeed = 0;
                }
                driveDutyCycle.Output = idleSpeed;
                mBottomShooter.setControl(driveDutyCycle);
                driveDutyCycle.Output = idleSpeed;
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

            case LOB:
                driveDutyCycle.Output = Constants.Shooter.bottomShooterLobSpeed;
                mBottomShooter.setControl(driveDutyCycle);
                driveDutyCycle.Output = Constants.Shooter.topShooterLobSpeed;
                mTopShooter.setControl(driveDutyCycle);
                break;

            default:
                break;
        }
    }

    public void setShooterPosition(ShootPosition pos) 
    {
        shooterMode = pos;
    }

    public ShootPosition getShootPosition() 
    {
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
    public boolean rpmWithinTolerance(double minShooterRPS) 
    {
        if (minShooterRPS == Constants.Shooter.lobVelocityTolerance) 
        {
            SmartDashboard.putString("SHOOTREADY State", "LOB");
            return mTopShooter.getVelocity().getValueAsDouble() < minShooterRPS;
        } 
        else 
        {
            SmartDashboard.putString("SHOOTREADY State", "SPEAKER");
            return mTopShooter.getVelocity().getValueAsDouble() > minShooterRPS;
        }
    }

    @Override
    public void periodic() 
    {
        // Prints info to Smart Dashboard
        SmartDashboard.putNumber("Top Shooter RPS", mTopShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Bottom Shooter RPS", mBottomShooter.getVelocity().getValueAsDouble());
    }

}