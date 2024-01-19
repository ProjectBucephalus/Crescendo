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
    private static TalonFX intakeArmMotor = new TalonFX(Constants.intakeArmMotorID);
    private static CANSparkMax m_intake = new CANSparkMax(8, CANSparkLowLevel.MotorType.kBrushed);
    static DigitalInput outLimitSwitch = new DigitalInput(Constants.outSwitchID);
    static DigitalInput inLimitSwitch = new DigitalInput(Constants.inSwitchID);
    double inLimit;
    double outLimit;

    public Intake() 
    {
        intakeArmMotor.setPosition(0);
    }

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

    public double angleToEncoderPosition(double targetAngle) 
    {
        targetAngle += Constants.shooterAngleOffset;
        targetAngle *= Constants.shooterGearRatios;
        
        return targetAngle;
    }

    public static void setIntakeDown()
    {
        if (inLimitSwitch.get())
        {
            intakeArmMotor.set(0);
        } else {
            intakeArmMotor.set(-0.20);
        }
    }

    public static void setIntakeUp()
    {
       if (outLimitSwitch.get()) 
       {
            intakeArmMotor.set(0);
       } else {
            intakeArmMotor.set(0.20);
       }
    }
// when function is triggered it will check if the limit switch is pressed
    public static void setIntakeAngleStop()
    {intakeArmMotor.set(0);}

    public static void IntakeIn()
    {
        m_intake.set(1);
    }

    public static void IntakeStop()
    {
        m_intake.set(0);
    }
    public static void intakeOut()
    {
        m_intake.set(-1);
    }
    //sets the speed for the intake

    public static void setIntakeStowed()
    {
        intakeArmMotor.set(1);
    }
}