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

    /**
     * Set Speed: sets the speed of the left and right climber
     * @authour 5985
     * @param speed the speed that the climber goes at
     */
    public enum ClimberPosition {STOWED, DEPLOYED};

    public Climber() 
    {}

    public void setSpeed(double speed) 
    {
        mLeftClimber.set(speed);
        mRightClimber.set(-speed);
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
