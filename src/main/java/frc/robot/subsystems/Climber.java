package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IDConstants;

/**
 * The Subsystem for the climber. Handles all climber movements as well as the RoboWrangler/Buddy Climber
 * @author 5985
 */
public class Climber extends SubsystemBase 
{
    // Declarations of all the motor controllers
    public TalonFX mLeftClimber = new TalonFX(IDConstants.Climber.mLeftClimbID);
    public TalonFX mRightClimber = new TalonFX(IDConstants.Climber.mRightClimbID);
    public TalonFX mBuddyClimb = new TalonFX(IDConstants.Climber.mBuddyClimbID);

    public DigitalInput leftClimberSwitch = new DigitalInput(5);
    public DigitalInput rightClimberSwitch = new DigitalInput(6);
    
    public static TalonFXConfiguration leftClimbMotorFXConfig = new TalonFXConfiguration();
    public static TalonFXConfiguration rightClimbMotorFXConfig = new TalonFXConfiguration();

    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public boolean isLocked = true;
    public boolean leftCalibrated = true;
    public boolean rightCalibrated = true;

    public Climber() {
        leftClimbMotorFXConfig.Slot0.kP = 100;
        leftClimbMotorFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftClimbMotorFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;  
        mLeftClimber.getConfigurator().apply(leftClimbMotorFXConfig);
        mLeftClimber.getConfigurator().setPosition(0);

        rightClimbMotorFXConfig.Slot0.kP = 100;
        rightClimbMotorFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;  
        rightClimbMotorFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;        
        mRightClimber.getConfigurator().apply(rightClimbMotorFXConfig);
        mRightClimber.getConfigurator().setPosition(0);
    }
    

    /** 
     * Enum representing the position of the climber 
     * @author 5985
     */
    public enum ClimberPosition 
    {
        UP,
        DOWN,
        STOPPED
    };

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
    };

    /** 
     * Sets the position of both climbers.
     * @param pos The rotations of the climber motor. 0 is stow, ~30 is extended
     * @author 5985
     */
    public void setPosition(double pos) 
    {   
        mLeftClimber.setControl(anglePosition.withPosition(pos))
                .withLimitReverseMotion(leftClimberSwitch.get());
        mRightClimber.setControl(anglePosition.withPosition(pos))
                .withLimitReverseMotion(rightClimberSwitch.get());
    }

    /** 
     * Sets the position of the climber, based on an input enum 
     * @param pos An instance of the ClimberPosition enum
     * @author 5985
     */
    public void setClimberPosition(ClimberPosition pos) 
    {
        if (!isLocked)
        {
            switch (pos) 
            {
                case UP:
                    setPosition(Constants.Climber.climberUpPos);
                    break;
                case DOWN:
                    setPosition(Constants.Climber.climberDownPos);
                    break;
                case STOPPED:
                    mLeftClimber.set(0);
                    mRightClimber.set(0);
                    break;
                default:
                    break;
            }
        }
        else
        {
            mLeftClimber.set(0);
            mRightClimber.set(0);
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
                isLocked = true;
                break;
            case UNLOCKED:
                isLocked = false;
                break;
            default:
                break;
        }
        //setClimberPosition(ClimberPosition.STOPPED);
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


    public boolean getLeftLimit() {
        return leftClimberSwitch.get();
    }


    public boolean getRightLimit() {
        return rightClimberSwitch.get();
    }

    /**
     * Gets the position of the climber in radians
     * @return The position of the climber in radians
     * @author 5985
     */
    public double getPosition() 
    {
        return (mLeftClimber.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic() 
    {
        SmartDashboard.putNumber("leftClimberPosition", mLeftClimber.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("rightClimberPosition", mRightClimber.getPosition().getValueAsDouble());

        SmartDashboard.putBoolean("leftClimberSwitch", getLeftLimit());
        SmartDashboard.putBoolean("RightClimberSwitch", getRightLimit());
        
        SmartDashboard.putBoolean("Climber Locked?", isLocked);

        if (isLocked) 
        {
            setPosition(0);
        }

        if (getLeftLimit() && !leftCalibrated) 
        {
            mLeftClimber.getConfigurator().setPosition(0);
            leftCalibrated = true;
            mLeftClimber.set(0);
        } 
        else if (!getLeftLimit() && leftCalibrated) 
        {
            leftCalibrated = false;
        }

        if (getRightLimit() && !rightCalibrated) 
        {
            mRightClimber.getConfigurator().setPosition(0);
            rightCalibrated = true;
            mRightClimber.set(0);
        } 
        else if (!getRightLimit() && rightCalibrated) 
        {
            rightCalibrated = false;
        }
    }
}
