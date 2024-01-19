package frc.robot.subsystems;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;

public class Intake extends SubsystemBase 
{
    TalonFX intakeArmMotor = new TalonFX(Constants.intakeArmMotorID);
    DigitalInput outLimitSwitch = new DigitalInput(Constants.outSwitchID);
    DigitalInput inLimitSwitch = new DigitalInput(Constants.inSwitchID);
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
}
