package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * the climber subsystem
 * @author 5985
 */
public class Climber extends SubsystemBase 
{

    public TalonFX mLeftClimber = new TalonFX(Constants.Climber.mLeftClimbID);
    public TalonFX mRightClimber = new TalonFX(Constants.Climber.mRightClimbID);
    public TalonFX mBuddyClimb = new TalonFX(Constants.Intake.mBuddyClimbID);

    public Climber() { 
        
    }

    public enum ClimberPosition {
        UP,
        DOWN,
    }
    public enum BuddyClimbPosition {
        RUNNING,
        STOPPED,
    };

    public enum ClimberStatus {
        LOCKED,
        UNLOCKED
    }

    private ClimberStatus climberStatus;
    private ClimberPosition climberPosition;

    public void setSpeed(double speed) 
    {
        mLeftClimber.set(speed);
        mRightClimber.set(-speed);
    }

    public void setPosition(ClimberPosition pos) {
        switch (pos) {
            case UP:
                
                break;
            case DOWN:
                
                break;
        
            default:
                break;
        }
    }
    
    public void setStatus(ClimberStatus status) {
        switch (status) {
            case LOCKED:
                
                break;
            case UNLOCKED:
                
                break;
        
            default:
                break;
        }
    }

    public void setBuddyClimb(BuddyClimbPosition status) {
        switch (status) {
            case RUNNING:
                mBuddyClimb.set(-1);
                break;
            case STOPPED:
                mBuddyClimb.set(0);
                break;
        
            default:
                break;
        }
    }

    /**
     * gets the position of the climber in radians
     * @return the position of the climber in radians
     */
    public double getPosition() 
    {
        SmartDashboard.putNumber("ClimberPosition", mLeftClimber.getPosition().getValueAsDouble());
        return (mLeftClimber.getPosition().getValueAsDouble());
    }

    
}
