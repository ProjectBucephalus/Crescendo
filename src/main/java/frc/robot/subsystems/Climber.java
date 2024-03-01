package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The Subsystem for the climber. Handles all climber movements as well as the RoboWrangler/Buddy Climber
 * @author 5985
 */
public class Climber extends SubsystemBase 
{
    // Declarations of all the motor controllers
    public TalonFX mLeftClimber = new TalonFX(Constants.Climber.mLeftClimbID);
    public TalonFX mRightClimber = new TalonFX(Constants.Climber.mRightClimbID);
    public TalonFX mBuddyClimb = new TalonFX(Constants.Intake.mBuddyClimbID);

    public Climber() 
    {}

    /** 
     * Enum representing the position of the climber 
     * @author 5985
     */
    public enum ClimberPosition 
    {
        UP,
        DOWN,
    }

    /** 
     * Enum representing the status of the RoboWrangler (Spinning/Not spinning)
     * @author 5985
     */
    public enum BuddyClimbPosition 
    {
        RUNNING,
        STOPPED,
    };

    /** 
     * Enum representing the status of the Climber
     * @author 5985
     */
    public enum ClimberStatus 
    {
        LOCKED,
        UNLOCKED
    }

    private ClimberStatus climberStatus; // Unused
    private ClimberPosition climberPosition; // Unused

    /** 
     * Sets the speeds of both climber sides, accounting for inversion
     * @author 5985
     */
    public void setSpeed(double speed) 
    {
        mLeftClimber.set(speed);
        mRightClimber.set(-speed);
    }

    /** 
     * Sets the position of the climber, based on an input enum 
     * @param pos An instance of the ClimberPosition enum
     * @author 5985
     */
    public void setPosition(ClimberPosition pos) 
    {
        switch (pos) 
        {
            case UP:
                
                break;
            case DOWN:
                
                break;
            default:
                break;
        }
    }
    
    /** 
     * Sets the status of the climber (Locked/Unlocked), based on an input enum 
     * @param status An instance of the ClimberStatus enum
     * @author 5985
     */
    public void setStatus(ClimberStatus status) 
    {
        switch (status) 
        {
            case LOCKED:
                
                break;
            case UNLOCKED:
                
                break;
            default:
                break;
        }
    }

    /** 
     * Sets the status of the RoboWrangler (Spinning/Not spinning), based on an input enum 
     * @param status An instance of the BuddyClimbPosition enum
     * @author 5985
     */
    public void setBuddyClimb(BuddyClimbPosition status) 
    {
        switch (status) 
        {
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
     * Gets the position of the climber in radians
     * @return The position of the climber in radians
     * @author 5985
     */
    public double getPosition() 
    {
        SmartDashboard.putNumber("ClimberPosition", mLeftClimber.getPosition().getValueAsDouble());
        return (mLeftClimber.getPosition().getValueAsDouble());
    }

    
}
