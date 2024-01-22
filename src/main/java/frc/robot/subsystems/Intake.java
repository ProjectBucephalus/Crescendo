package frc.robot.subsystems;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Intake extends SubsystemBase 
{
    //motors
    public TalonFX mLeftPivot = new TalonFX(Constants.Intake.mLeftPivotID);
    public TalonFX mRightPivot = new TalonFX(Constants.Intake.mRightPivotID);

    public TalonFX mIntake = new TalonFX(Constants.Intake.mIntakeID);
    public TalonFX mFlap = new TalonFX(Constants.Intake.mFlapID); //Victor SPX

    public TalonFX mTopShooter = new TalonFX(Constants.Shooter.mTopShooterID);
    public TalonFX mBottomShooter = new TalonFX(Constants.Shooter.mBottomShooterID);
    
    //limit switches
    public DigitalInput leftDeploySwitch = new DigitalInput(Constants.Intake.leftOutSwitchID);
    public DigitalInput leftStowSwitch = new DigitalInput(Constants.Intake.leftInSwitchID);
    public DigitalInput rightDeploySwitch = new DigitalInput(Constants.Intake.rightOutSwitchID);
    public DigitalInput rightStowSwitch = new DigitalInput(Constants.Intake.rightInSwitchID);
    //limits as checked during calibration, to account for encoder drift
    double intakeStowLimitPos;
    double intakeDeployLimitPos;

    public Intake() {
        mLeftPivot.setPosition(0);
        mRightPivot.setPosition(0);
    }

    /* moves the arm to a set position, with safety measures */
    public void moveArm(double armAngle) 
    {
        armAngle += intakeStowLimitPos;
        if (leftStowSwitch.get() || rightStowSwitch.get()) 
        {
            setArmMotorSpeeds(0);
            intakeStowLimitPos = mLeftPivot.getPosition().getValueAsDouble();
        } 
        else if (leftDeploySwitch.get() || rightDeploySwitch.get()) 
        {
            setArmMotorSpeeds(0);
            intakeDeployLimitPos = mLeftPivot.getPosition().getValueAsDouble();
        }
    }

    /* calibrates the limits */
    public void calibrateLimits() 
    {    
        while (!leftStowSwitch.get() || !rightStowSwitch.get()) 
        {  
            setArmMotorSpeeds(0.1);
        }
        setArmMotorSpeeds(0);
        intakeStowLimitPos = mLeftPivot.getPosition().getValueAsDouble();
        
        while (!leftDeploySwitch.get() || !rightDeploySwitch.get()) 
        {  
            setArmMotorSpeeds(-0.1);
        }
        setArmMotorSpeeds(0);
        intakeDeployLimitPos = mLeftPivot.getPosition().getValueAsDouble();
        System.out.println("Calibration done");
    }

    /* takes in a desired angle of the intake, and outputs the motor position needed, accounting for gearing */
    public double angleToEncoderPosition(double targetAngle) 
    {
        targetAngle += Constants.shooterAngleOffset;
        targetAngle *= Constants.shooterGearRatios;
        
        return targetAngle;
    }

    public void setArmMotorSpeeds(double speed) {
        mLeftPivot.set(speed);
        mRightPivot.set(-speed);
    }

    /* drives intake arm down */
    public void setIntakeDown()
    {
        if (leftStowSwitch.get() || leftStowSwitch.get())
        {
            setArmMotorSpeeds(0);
        } 
        else 
        {
            setArmMotorSpeeds(-0.20);
        }
    }

    /* drives intake arm up */
    public void setIntakeUp()
    {
       if (leftDeploySwitch.get() || rightDeploySwitch.get()) 
       {
            setArmMotorSpeeds(0);
       } 
       else 
       {
            setArmMotorSpeeds(0.2);
       }
    }
    
    /* stops intake arm motion */
    public void setIntakeAngleStop()
    {
        setArmMotorSpeeds(0);
    }

    /* drives the intake to suck pieces in */
    public void intakeIn()
    {
        mIntake.set(1);
    }

    /* stops the intake */
    public void intakeStop()
    {
        mIntake.set(0);
    }
    
    /* drives the intake to spit pieces out */
    public void intakeOut()
    {
        mIntake.set(-1);
    }
    
    //commented out for safety's sake. same with reference to it in IntakeStowed file
     
    public void setIntakeStowed()
    {
        while (leftStowSwitch.get() || !rightStowSwitch.get()) 
        {  
            setArmMotorSpeeds(0.2);;  
        }
        mIntake.stopMotor();
    }

    /* stops the intake arm moving */
    public void intakeArmStop()
    {
        setArmMotorSpeeds(0);
    }

    /* sets shooter to full speed */
    public void spinShooter() 
    {
        mTopShooter.set(1);
        mBottomShooter.set(1);
    }

    /* stops shooter */
    public void stopShooter() 
    {
        mTopShooter.set(0);
        mBottomShooter.set(0);
    }
    
    /* sets shooter to idle speed */
    public void idleShooter() 
    {
        mTopShooter.set(0.5);
        mBottomShooter.set(0.5);
    }

    /* opens the feed flap */
    public void openFlap() 
    {
        while (mFlap.getTorqueCurrent().getValueAsDouble() <= Constants.Intake.FlapMaxCurrent) 
        {
            mFlap.set(0.5);
        }
        mFlap.set(0);
    }

    /* closes the feed flap */
    public void closeFlap () 
    {
        while (mFlap.getStatorCurrent().getValueAsDouble() <= Constants.Intake.FlapMaxCurrent) 
        {
            mFlap.set(-0.5);
        }
        mFlap.set(0);
    }
}