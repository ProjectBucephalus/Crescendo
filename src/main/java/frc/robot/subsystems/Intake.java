package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.lib.math.Conversions;
import frc.robot.CTREConfigs;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Intake extends SubsystemBase {
    // motors
    public TalonFX mLeftPivot;
    public TalonFX mRightPivot;
    private final PositionVoltage anglePosition = new PositionVoltage(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);

    public TalonFX mIntake = new TalonFX(Constants.Intake.mIntakeID);
    public VictorSPX mFlap = new VictorSPX(Constants.Intake.mFlapID);

    public TalonFX mTopShooter = new TalonFX(Constants.Shooter.mTopShooterID);
    public TalonFX mBottomShooter = new TalonFX(Constants.Shooter.mBottomShooterID);

    // limit switches
    public DigitalInput leftDeploySwitch = new DigitalInput(Constants.Intake.leftOutSwitchID);
    public DigitalInput leftStowSwitch = new DigitalInput(Constants.Intake.leftInSwitchID);
    public DigitalInput rightDeploySwitch = new DigitalInput(Constants.Intake.rightOutSwitchID);
    public DigitalInput rightStowSwitch = new DigitalInput(Constants.Intake.rightInSwitchID);
    // limits as checked during calibration, to account for encoder drift
    double intakeStowLimitPos;
    double intakeDeployLimitPos;

    public enum IntakePosition {
        STOWED,
        DEPLOYED,
    };

    public enum FlapPosition {
        OPEN,
        CLOSED,
    };

    public Intake() {

        SmartDashboard.putNumber("topShooterSpeed", 0.1);
        SmartDashboard.putNumber("bottomShooterSpeed", 0.1);
        SmartDashboard.putNumber("pivotPosition", 1.2);

        mLeftPivot = new TalonFX(Constants.Intake.mLeftPivotID);
        mLeftPivot.getConfigurator().apply(CTREConfigs.leftArmMotorFXConfig);
        mLeftPivot.getConfigurator().setPosition(0);

        mRightPivot = new TalonFX(Constants.Intake.mRightPivotID);
        mRightPivot.getConfigurator().apply(CTREConfigs.rightArmMotorFXConfig);
        mRightPivot.getConfigurator().setPosition(0);
    }

    public void setPosition(IntakePosition position) {
        switch (position) {
            case STOWED:
                moveArmToAngle(0);
                break;
            case DEPLOYED:
                moveArmToAngle(SmartDashboard.getNumber("pivotPosition", 1.2));
                break;
        }
    }

    /* moves the arm to a set position, In radians */
    public void moveArmToAngle(double armAngle) { // TODO add limit switch protections
        mLeftPivot.setControl(anglePosition.withPosition(armAngle));
        mRightPivot.setControl(anglePosition.withPosition(armAngle));
    }

    public double getArmPos() {
        return mRightPivot.getPosition().getValueAsDouble();
    }

    public void setArmMotorSpeeds(double speed) {
        mLeftPivot.set(speed);
        mRightPivot.set(-speed);
    }

    public void setIntakeSpeed(double speed) {
        mIntake.set(speed);
    }

    // commented out for safety's sake. same with reference to it in IntakeStowed
    // file

    public void setIntakeStowed() 
    {
        if (leftStowSwitch.get() || !rightStowSwitch.get()) 
        {
            setArmMotorSpeeds(0.2);
        }
        else
        {
            mIntake.stopMotor();
        }
    }

    /* sets shooter to full speed */
    public void spinShooter() {
        // mTopShooter.set(0.3);
        // mBottomShooter.set(0.07);
        double bottomSpeed = SmartDashboard.getNumber("bottomShooterSpeed", 0);
        double topSpeed = SmartDashboard.getNumber("topShooterSpeed", 0);

        driveDutyCycle.Output = bottomSpeed;
        mBottomShooter.setControl(driveDutyCycle);
        driveDutyCycle.Output = topSpeed;
        mTopShooter.setControl(driveDutyCycle);
        SmartDashboard.putNumber("bottomShooterSpeed", bottomSpeed);
        SmartDashboard.putNumber("topShooterSpeed", topSpeed);
    }

    /* stops shooter */
    public void stopShooter() {
        mTopShooter.set(0);
        mBottomShooter.set(0);
    }

    /* sets shooter to idle speed */
    public void idleShooter() {
        mTopShooter.set(Constants.Shooter.shooterIdleSpeed);
        mBottomShooter.set(Constants.Shooter.shooterIdleSpeed);
    }

    public void setFlapSpeed(double speed) {
        mFlap.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public void setFlapPosition (FlapPosition pos) {
        switch (pos) {
            case OPEN:
                
                break;
        
            case CLOSED:
            
                break;
        }
    }
}