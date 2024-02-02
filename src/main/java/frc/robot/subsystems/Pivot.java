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

public class Pivot extends SubsystemBase {
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

    public TalonFX mBuddyClimb = new TalonFX(Constants.Intake.mBuddyClimbID);

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
        AMP,
        TRAP,
        SPEAKER,
    };

    public enum FlapPosition {
        OPEN,
        CLOSED,
    };

    public Pivot() {

    }

    public void setPosition(IntakePosition position) {
        switch (position) { 
            case STOWED:
                moveArmToAngle(0);
                break;
            case DEPLOYED:
                moveArmToAngle(SmartDashboard.getNumber("pivotPosition", 1.162300));
                break;
            case AMP:
                moveArmToAngle(Constants.Intake.pivotAmpPos);
            case TRAP:
                moveArmToAngle(Constants.Intake.trapPos);
                break;
            case SPEAKER:
                //TODO April tag stuff
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
    // commented out for safety's sake. same with reference to it in IntakeStowed
    // file

    public void setIntakeStowed() {
        while (leftStowSwitch.get() || !rightStowSwitch.get()) {
            setArmMotorSpeeds(0.2);
        }
        mIntake.stopMotor();
    }

}