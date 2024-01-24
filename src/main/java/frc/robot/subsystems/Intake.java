package frc.robot.subsystems;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.CTREConfigs;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Intake extends SubsystemBase 
{
    //motors
    public TalonFX mLeftPivot;
    public TalonFX mRightPivot;
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public TalonFX mIntake = new TalonFX(Constants.Intake.mIntakeID);
    public VictorSPX mFlap = new VictorSPX(Constants.Intake.mFlapID);

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
        mLeftPivot = new TalonFX(Constants.Intake.mLeftPivotID);
        mLeftPivot.getConfigurator().apply(CTREConfigs.leftArmMotorFXConfig);
        mLeftPivot.setPosition(0);

        mRightPivot = new TalonFX(Constants.Intake.mRightPivotID);
        mRightPivot.getConfigurator().apply(CTREConfigs.rightArmMotorFXConfig);
        mRightPivot.setPosition(0);
    }

    /* moves the arm to a set position, In radians */
    public void moveArmToAngle(double armAngle) { // TODO add limit switch protections
        mLeftPivot.setControl(anglePosition.withPosition(armAngle));
        mRightPivot.setControl(anglePosition.withPosition(armAngle));
    }

    public void setArmMotorSpeeds(double speed) {
        mLeftPivot.set(speed);
        mRightPivot.set(-speed);
    }

    public void setIntakeSpeed(double speed) {
        mIntake.set(speed);
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

    public void setFlapSpeed(double speed) {
        mFlap.set(VictorSPXControlMode.PercentOutput, speed);
    }
}