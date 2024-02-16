package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
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

import javax.print.attribute.standard.Destination;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Pivot extends SubsystemBase {
    // motors
    public TalonFX mLeftPivot;
    public TalonFX mRightPivot;

    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public TalonFX mBuddyClimb = new TalonFX(Constants.Intake.mBuddyClimbID);

    //public ArmFeedforward feedforward = new ArmFeedforward(0.9, 0.45, 0.82, 0); // not implimented yet, and may not be calibrated yet

    // limit switches
    public DigitalInput leftDeploySwitch  = new DigitalInput(Constants.Intake.leftOutSwitchID);
    public DigitalInput leftStowSwitch    = new DigitalInput(Constants.Intake.leftInSwitchID);
    public DigitalInput rightDeploySwitch = new DigitalInput(Constants.Intake.rightOutSwitchID);
    public DigitalInput rightStowSwitch   = new DigitalInput(Constants.Intake.rightInSwitchID);
    // limits as checked during calibration, to account for encoder drift
    double intakeStowLimitPos;
    double intakeDeployLimitPos;

    double desiredAngle = 0;

    /** Flagged true when pivot is calibrated to stowed position at start of match */
    boolean stowCalibrated = false;

    public enum PivotPosition {
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
        SmartDashboard.putNumber("deployPosition", -95);
        SmartDashboard.putNumber("ampPosition", -60);

        mLeftPivot = new TalonFX(Constants.Intake.mLeftPivotID);
        mLeftPivot.getConfigurator().apply(CTREConfigs.leftPivotMotorFXConfig);
        mLeftPivot.getConfigurator().setPosition(0);

        mRightPivot = new TalonFX(Constants.Intake.mRightPivotID);
        mRightPivot.getConfigurator().apply(CTREConfigs.rightPivotMotorFXConfig);
        mRightPivot.getConfigurator().setPosition(0);

        mRightPivot.setControl(new Follower(mLeftPivot.getDeviceID(), true));
    }

    public void setPosition(PivotPosition position) {
        switch (position) { 
            case STOWED:
                moveArmToAngle(0); // TODO Ensure this is checking limit switches
                break;
            case DEPLOYED:
                moveArmToAngle(SmartDashboard.getNumber("deployPosition", -95));
                break;
            case AMP:
                moveArmToAngle(SmartDashboard.getNumber("ampPosition", -60));
                break;
            case TRAP:
                moveArmToAngle(Constants.Intake.pivotTrapPos);
                break;
            case SPEAKER:
                moveArmToAngle(-75); // TODO Hard coded for testing
                break;
        }
    }

    /** moves the arm to a set position, in degrees
     * TODO Something is seriously wrong with the limit switch use and MUST be resolved
     * TODO Angle calculations and conversion need to be recalibrated to use reasonable real-world angles
     * @param armAngle Degrees - The angle to move the arm to in degrees. Negative numbers to intake pos. Positive to stow pos.
     */
    private void moveArmToAngle(double armAngle) { // TODO add limit switch protections
        desiredAngle = armAngle;
        SmartDashboard.putNumber("desiredAngle", desiredAngle);
        mLeftPivot.setControl
        (
            anglePosition.withPosition((desiredAngle/360))
                .withLimitReverseMotion(leftDeploySwitch.get())
                .withLimitReverseMotion(rightDeploySwitch.get())

                .withLimitForwardMotion(leftStowSwitch.get())
                .withLimitForwardMotion(rightStowSwitch.get())
        );
    }


    /**
     * 
     * @param angle the desired angle to move the arm to when the setPosition() method is called with the correct enum.
     * 
     * for example, if we want to align to the speaker, we would call this metod and pass in the desired angle and call setPosition() with 
     * PivotPosition.SPEAKER.
     */
    public void setDesiredPostion(Double angle) {
        desiredAngle = angle;
    }

    public double getArmPos() {
        return ((mRightPivot.getPosition().getValueAsDouble() * 360)+(mLeftPivot.getPosition().getValueAsDouble() * 360))/2;
    }

    /**
     * Manually sets the movement of the intake position, respecting limitswitches. 
     * Resets desiredAngle so manual position is held. 
     * @param speed Double speed of motor [-1..1], posative to Deploy (Confirm this)
     * @author 5985
     * @author Alec
     */
    public void setArmMotorSpeeds(double speed) 
    {
        if 
          ( 
            (speed < 0 && !leftDeploySwitch.get() && !rightDeploySwitch.get())
              ||
            (speed > 0 && !leftStowSwitch  .get() && !rightStowSwitch  .get())
          )
        {
            mLeftPivot.set(speed);
            //mRightPivot.set(speed); // mRightPivot is set as reversed follower of mLeftPivot
            desiredAngle = getArmPos();
        }
        else
        {
            mLeftPivot.set(0);
            //mRightPivot.set(0); // mRightPivot is set as reversed follower of mLeftPivot
        }
    }
    
    // commented out for safety's sake. same with reference to it in IntakeStowed file

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ReportedPivotPosition", getArmPos());
        SmartDashboard.putNumber("PivotError", desiredAngle-getArmPos());
        SmartDashboard.putBoolean("leftDeploySwitch", leftDeploySwitch.get());
        SmartDashboard.putBoolean("leftStowSwitch", leftStowSwitch.get());
        SmartDashboard.putBoolean("rightDeploySwitch", rightDeploySwitch.get());
        SmartDashboard.putBoolean("rightStowSwitch", rightStowSwitch.get());
        
        if (leftStowSwitch.get() && !stowCalibrated) 
        {
            mLeftPivot.getConfigurator().setPosition(0);
            stowCalibrated = true;
        }
        if (rightStowSwitch.get() && !stowCalibrated) 
        {
            mRightPivot.getConfigurator().setPosition(0);
            stowCalibrated = true;
        }
        if (leftDeploySwitch.get()) 
        {
            mLeftPivot.getConfigurator().setPosition(Units.degreesToRotations(-100));
        }
        if (rightDeploySwitch.get()) 
        {
            mRightPivot.getConfigurator().setPosition(Units.degreesToRotations(-100));
        }
    }

}