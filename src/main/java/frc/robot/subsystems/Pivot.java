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
import frc.robot.SwerveModule;
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

    public ArmFeedforward feedforward = new ArmFeedforward(0.9, 0.45, 0.82, 0);

    // limit switches
    public DigitalInput leftDeploySwitch = new DigitalInput(Constants.Intake.leftOutSwitchID);
    public DigitalInput leftStowSwitch = new DigitalInput(Constants.Intake.leftInSwitchID);
    public DigitalInput rightDeploySwitch = new DigitalInput(Constants.Intake.rightOutSwitchID);
    public DigitalInput rightStowSwitch = new DigitalInput(Constants.Intake.rightInSwitchID);
    // limits as checked during calibration, to account for encoder drift
    double intakeStowLimitPos;
    double intakeDeployLimitPos;

    double desiredAngle = 0;

    // Limit Switches for backlash
    boolean limitSwitchFlags[];

    public DigitalInput limitSwitches[];

    boolean leftDeployPressed = false;
    boolean leftStowPressed = false;
    boolean rightDeployPressed = false;
    boolean rightStowPressed = false;

    /**
     * Set to false on robot init. This is set to true once any limit is hit to
     * ensure safety. Once the position of the pivot is known, we no longer
     * calibrate with the stowed limits rather use the deployed limts for backlash
     * calibration.
     */
    boolean isCalibrated = false;

    public enum PivotPosition {
        STOWED,
        DEPLOYED,
        AMP,
        TRAP,
        SPEAKER,
        AMP_MANUAL,
        TRAP_MANUAL,
        SPEAKER_MANUAL
    };

    public enum FlapPosition {
        OPEN,
        CLOSED,
    };

    public Pivot() {
        SmartDashboard.putNumber("pivotPosition", -90);
        SmartDashboard.putNumber("ampPosition", -45);

        mLeftPivot = new TalonFX(Constants.Intake.mLeftPivotID);
        mLeftPivot.getConfigurator().apply(CTREConfigs.leftPivotMotorFXConfig);
        mLeftPivot.getConfigurator().setPosition(0);

        mRightPivot = new TalonFX(Constants.Intake.mRightPivotID);
        mRightPivot.getConfigurator().apply(CTREConfigs.rightPivotMotorFXConfig);
        mRightPivot.getConfigurator().setPosition(0);

        limitSwitchFlags = new boolean[] {
                leftDeployPressed,
                leftStowPressed,
                rightDeployPressed,
                rightStowPressed
        };

        limitSwitches = new DigitalInput[] {
                leftDeploySwitch,
                leftStowSwitch,
                rightDeploySwitch,
                rightStowSwitch
        };

    }

    public void setPosition(PivotPosition position) {
        switch (position) {
            case STOWED:
                moveArmToAngle(Constants.Intake.pivotGearStowAngle);
                break;
            case DEPLOYED:
                moveArmToAngle(SmartDashboard.getNumber("pivotPosition", Constants.Intake.pivotGearDeployAngle));
                break;
            case AMP:
                moveArmToAngle(SmartDashboard.getNumber("ampPosition", 45));
                break;
            case TRAP:
                moveArmToAngle(Constants.Intake.trapPos);
                break;
            case SPEAKER:
                moveArmToAngle(desiredAngle);
                break;
            case AMP_MANUAL:
                moveArmToAngle(0);
                break;
            case TRAP_MANUAL:
                moveArmToAngle(0);
                break;
            case SPEAKER_MANUAL:
                moveArmToAngle(50);
                break;
        }
    }

    /**
     * moves the arm to a set position, In degrees
     * 
     * @param armAngle The angle to move the arm to in degrees. Negative numbers to
     *                 intake pos. Positive to stow pos.
     *                 Uses limits
     */
    private void moveArmToAngle(double armAngle) {
        desiredAngle = armAngle;
        SmartDashboard.putNumber("desiredAngle", armAngle);
        mLeftPivot.setControl(anglePosition.withPosition((armAngle / 360))
                .withLimitReverseMotion(leftDeploySwitch.get())
                .withLimitReverseMotion(rightDeploySwitch.get())

                .withLimitForwardMotion(leftStowSwitch.get())
                .withLimitForwardMotion(rightStowSwitch.get()));

        mRightPivot.setControl(new Follower(mLeftPivot.getDeviceID(), true)

        // .withLimitForwardMotion(rightDeploySwitch.get())
        // .withLimitForwardMotion(leftDeploySwitch.get())

        // .withLimitReverseMotion(rightStowSwitch.get())
        // .withLimitReverseMotion(leftStowSwitch.get())
        );
    }

    /**
     * 
     * @param angle the desired angle to move the arm to when the setPosition()
     *              method is called with the correct enum.
     * 
     *              for example, if we want to align to the speaker, we would call
     *              this metod and pass in the desired angle and call setPosition()
     *              with
     *              PivotPosition.SPEAKER.
     */
    public void setDesiredPostion(Double angle) {
        this.desiredAngle = angle;
    }

    public double getArmPos() {
        return ((mRightPivot.getPosition().getValueAsDouble() * 360)
                + (mLeftPivot.getPosition().getValueAsDouble() * 360)) / 2;
    }

    public void setArmMotorSpeeds(double speed) {
        mLeftPivot.set(speed);
        mRightPivot.set(speed);
    }
    // commented out for safety's sake. same with reference to it in IntakeStowed
    // file

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ReportedPivotPosition", getArmPos());
        SmartDashboard.putBoolean("leftDeploySwitch", leftDeploySwitch.get());
        SmartDashboard.putBoolean("leftStowSwitch", leftStowSwitch.get());
        SmartDashboard.putBoolean("rightDeploySwitch", rightDeploySwitch.get());
        SmartDashboard.putBoolean("rightStowSwitch", rightStowSwitch.get());

        SmartDashboard.putNumber("PivotError", desiredAngle - getArmPos());
        SmartDashboard.putBoolean("isCalibrated", isCalibrated);

        if (leftDeploySwitch.get()) {
            leftDeployPressed = true;
        }
        if (rightDeploySwitch.get()) {
            rightDeployPressed = true;
        }

        if (leftDeployPressed == true && !leftDeploySwitch.get()) {
            leftDeployPressed = false;
            mLeftPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotGearDeployAngle));
        }
        if (rightDeployPressed == true && !rightDeploySwitch.get()) {
            rightDeployPressed = false;
            mRightPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotGearDeployAngle));
        }

        if (leftStowSwitch.get() && !isCalibrated) {
            isCalibrated = true;
            mLeftPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotGearStowAngle));
        }

        if (rightStowSwitch.get() && !isCalibrated) {
            isCalibrated = true;
            mRightPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotGearStowAngle));
        }

        if (leftDeploySwitch.get() && !isCalibrated) {
            isCalibrated = true;
            mLeftPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotGearDeployAngle));
        }

        if (rightDeploySwitch.get() && !isCalibrated) {
            isCalibrated = true;
            mRightPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotGearDeployAngle));
        }

        for (int i = 0; i < limitSwitches.length; i++) {
            boolean isPressed = limitSwitches[i].get();
            if (isPressed) {

            } else {

            }
        }
        for (int i = 0; i < limitSwitchFlags.length; i++) {
            boolean isPressed = limitSwitchFlags[i];
            if (isPressed) {

            } else {

            }
        }
    }

}