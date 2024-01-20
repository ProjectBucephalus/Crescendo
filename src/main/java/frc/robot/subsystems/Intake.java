package frc.robot.subsystems;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;

public class Intake extends SubsystemBase 
{
    //motors
    private static TalonFX intakeArmMotor = new TalonFX(Constants.intakeArmMotorID);
    private static CANSparkMax m_intake = new CANSparkMax(8, CANSparkMax.MotorType.kBrushed);
    //limit switches
    public static DigitalInput outLimitSwitch = new DigitalInput(Constants.outSwitchID);
    public static DigitalInput inLimitSwitch = new DigitalInput(Constants.inSwitchID);
    //limits as checked during calibration, to account for encoder drift
    double inLimit;
    double outLimit;

    public Intake() 
    {
        intakeArmMotor.setPosition(0);
    }

    /* moves the arm to a set position, with safety measures */
    public void moveArm(double armAngle) 
    {
        armAngle += inLimit;
        intakeArmMotor.setPosition(armAngle);
        if (inLimitSwitch.get()) 
        {
            intakeArmMotor.stopMotor();
            inLimit = intakeArmMotor.getPosition().getValueAsDouble();
        } 
        else if (outLimitSwitch.get()) 
        {
            intakeArmMotor.stopMotor();
            outLimit = intakeArmMotor.getPosition().getValueAsDouble();
        }
    }

    /* calibrates the limits */
    public void calibrateLimits() 
    {    
        while (!inLimitSwitch.get()) 
        {  
            intakeArmMotor.set(0.1);  
        }
        intakeArmMotor.stopMotor();
        inLimit = intakeArmMotor.getPosition().getValueAsDouble();
        
        while (!outLimitSwitch.get()) 
        {  
            intakeArmMotor.set(-0.1);
        }
        intakeArmMotor.stopMotor();
        outLimit = intakeArmMotor.getPosition().getValueAsDouble();
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
            intakeArmMotor.set(0);
        } 
        else 
        {
            intakeArmMotor.set(-0.20);
        }
    }

    /* drives intake arm up */
    public static void setIntakeUp()
    {
       if (outLimitSwitch.get()) 
       {
            intakeArmMotor.set(0);
       } 
       else 
       {
            intakeArmMotor.set(0.20);
       }
    }
    
    /* stops intake arm motion */
    public static void setIntakeAngleStop()
    {
        intakeArmMotor.set(0);
    }

    /* drives the intake to suck pieces in */
    public static void IntakeIn()
    {
        m_intake.set(1);
    }

    /* stops the intake */
    public static void IntakeStop()
    {
        m_intake.set(0);
    }
    
    /* drives the intake to spit pieces out */
    public static void intakeOut()
    {
        m_intake.set(-1);
    }
    
    //commented out for safety's sake. same with reference to it in IntakeStowed file
     
    public static void setIntakeStowed()
    {
        while (!inLimitSwitch.get()) 
        {  
            intakeArmMotor.set(0.2);  
        }
        intakeArmMotor.stopMotor();
    }

    public static void intakeArmStop()
    {
        intakeArmMotor.stopMotor();
    }
    
}