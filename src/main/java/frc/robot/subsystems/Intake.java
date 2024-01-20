package frc.robot.subsystems;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;

public class Intake extends SubsystemBase 
{
    //motors
    private static TalonFX mIntakeArm = new TalonFX(Constants.mIntakeArmID);
    private static CANSparkMax mIntake = new CANSparkMax(Constants.mIntakeID, CANSparkMax.MotorType.kBrushed);
    private static TalonFX mShooter = new TalonFX(Constants.mShooterID);
    //limit switches
    public static DigitalInput outLimitSwitch = new DigitalInput(Constants.outSwitchID);
    public static DigitalInput inLimitSwitch = new DigitalInput(Constants.inSwitchID);
    //limits as checked during calibration, to account for encoder drift
    double inLimit;
    double outLimit;

    public Intake() 
    {
        mIntakeArm.setPosition(0);
    }

    /* moves the arm to a set position, with safety measures */
    public void moveArm(double armAngle) 
    {
        armAngle += inLimit;
        mShooter.setPosition(armAngle);
        if (inLimitSwitch.get()) 
        {
            mIntakeArm.stopMotor();
            inLimit = mIntakeArm.getPosition().getValueAsDouble();
        } 
        else if (outLimitSwitch.get()) 
        {
            mIntakeArm.stopMotor();
            outLimit = mIntakeArm.getPosition().getValueAsDouble();
        }
    }

    /* calibrates the limits */
    public void calibrateLimits() 
    {    
        while (!inLimitSwitch.get()) 
        {  
            mIntakeArm.set(0.1);  
        }
        mIntakeArm.stopMotor();
        inLimit = mIntakeArm.getPosition().getValueAsDouble();
        
        while (!outLimitSwitch.get()) 
        {  
            mIntakeArm.set(-0.1);
        }
        mIntakeArm.stopMotor();
        outLimit = mIntakeArm.getPosition().getValueAsDouble();
        System.out.println("Calibration done");
    }

    /* takes in a desired angle of the intake, and outputs the motor position needed, accounting for gearing */
    public double angleToEncoderPosition(double targetAngle) 
    {
        targetAngle += Constants.shooterAngleOffset;
        targetAngle *= Constants.shooterGearRatios;
        
        return targetAngle;
    }

    /* drives intake arm down */
    public static void setIntakeDown()
    {
        if (inLimitSwitch.get())
        {
            mIntakeArm.set(0);
        } 
        else 
        {
            mIntakeArm.set(-0.20);
        }
    }

    /* drives intake arm up */
    public static void setIntakeUp()
    {
       if (outLimitSwitch.get()) 
       {
            mIntakeArm.set(0);
       } 
       else 
       {
            mIntakeArm.set(0.20);
       }
    }
    
    /* stops intake arm motion */
    public static void setIntakeAngleStop()
    {
        mIntakeArm.set(0);
    }

    /* drives the intake to suck pieces in */
    public static void IntakeIn()
    {
        mIntake.set(1);
    }

    /* stops the intake */
    public static void IntakeStop()
    {
        mIntake.set(0);
    }
    
    /* drives the intake to spit pieces out */
    public static void intakeOut()
    {
        mIntake.set(-1);
    }
    
    //commented out for safety's sake. same with reference to it in IntakeStowed file
     
    public static void setIntakeStowed()
    {
       // mIntakeArm.setPosition(inLimit);
    }

    public static void intakeArmStop()
    {
        mIntakeArm.stopMotor();
    }

    public static void spinShooter() 
    {
        mShooter.set(1);
    }

    public static void stopShooter() 
    {
        mShooter.stopMotor();
    }
    
}