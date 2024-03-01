package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.CTREConfigs;

public class Pivot extends SubsystemBase {
    // motors
    public TalonFX mLeftPivot;
    public TalonFX mRightPivot;

    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public TalonFX mBuddyClimb = new TalonFX(Constants.Intake.mBuddyClimbID);

    // public ArmFeedforward feedforward = new ArmFeedforward(0.9, 0.45, 0.82, 0);
    // // not implimented yet, and may not be calibrated yet

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
        SmartDashboard.putNumber("deployPosition", -95);
        SmartDashboard.putNumber("ampPosition", -60);

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

    /**
     * Sets the position of the pivot based on an enum
     * 
     * @param position The desired position, as an enum
     * @author 5985
     * @author Aidan
     */
    public void setPosition(PivotPosition position) {
        SmartDashboard.putString("Pivot Position Status", position.name());
        switch (position) {
            case STOWED:
                moveArmToAngle(Constants.Intake.pivotStowPos); // TODO Ensure this is checking limit switches
                break;
            case DEPLOYED:
                moveArmToAngle(Constants.Intake.pivotDeployPos);
                break;
            case AMP:
                moveArmToAngle(Constants.Intake.pivotAmpPos);
                break;
            case TRAP:
                moveArmToAngle(Constants.Intake.pivotTrapPos);
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
     * moves the arm to a set position, in degrees
     * TODO Something is seriously wrong with the limit switch use and MUST be
     * resolved
     * TODO Angle calculations and conversion need to be recalibrated to use
     * reasonable real-world angles
     * 
     * @param inputAngle The real-world angle to move the arm to in degrees. 
     *                   Intake Posative, 
     *                   Using limits switches.
     */
    public void moveArmToAngle(double inputAngle) {
        desiredAngle = inputAngle;
        SmartDashboard.putNumber("desiredAngle", desiredAngle);
        // double motorAngle = -(desiredAngle - Constants.Intake.pivotOffsetForZero);
        mLeftPivot.setControl(
                anglePosition.withPosition((inputAngle / 360))

                        .withLimitReverseMotion(leftStowSwitch.get())
                        .withLimitReverseMotion(rightStowSwitch.get())

                        .withLimitForwardMotion(leftDeploySwitch.get())
                        .withLimitForwardMotion(rightDeploySwitch.get())
        );
        mRightPivot.setControl(new Follower(mLeftPivot.getDeviceID(), true)); 
        // CTREConfigs already has Left and Right use opposite directions

    }

    // .withLimitForwardMotion(rightDeploySwitch.get())
    // .withLimitForwardMotion(leftDeploySwitch.get())

    // .withLimitReverseMotion(rightStowSwitch.get())
    // .withLimitReverseMotion(leftStowSwitch.get())

    /**
     * 
     * @param angle the desired angle to move the arm to when the setPosition()
     *              method is called with the correct enum.
     * 
     *              for example, if we want to align to the speaker, we would call
     *              this method and pass in the desired angle and call setPosition()
     *              with
     *              PivotPosition.SPEAKER.
     */
    public void setDesiredPostion(Double angle) {
        desiredAngle = angle;
    }

    /**
     * Gets the pivot position as the average of the two motors
     * @return Pivot position degrees
     */
    public double getPivotPos()
    {
        // motor.getPosition() returns rotations: convert to degrees and return average.
        // return ((mRightPivot.getPosition().getValueAsDouble() * 360)
        //         + (mLeftPivot.getPosition().getValueAsDouble() * 360)) / 2;
        return mRightPivot.getPosition().getValueAsDouble();
    }

    public boolean angleWithinTolerance() {
        return Math.abs(desiredAngle - getPivotPos()) < Constants.Intake.ANGLE_TOLERANCE_DEGREE;
    }

    /**
     * Manually sets the movement of the intake position, respecting limitswitches. 
     * Resets desiredAngle so manual position is held.
     * 
     * @param speed Double speed of motor [-1..1], negative to Deploy
     * @author 5985
     * @author Alec
     */
    public void setArmMotorSpeeds(double speed) 
    {
        if ((speed < 0 && !leftDeploySwitch.get() && !rightDeploySwitch.get())
              ||
            (speed > 0 && !leftStowSwitch.get() && !rightStowSwitch.get())) 
        {
            mLeftPivot.set(speed);
            // mRightPivot.set(speed); // mRightPivot is set as reversed follower of mLeftPivot
            desiredAngle = getPivotPos();
        } 
        else 
        {
            mLeftPivot.set(0);
            // mRightPivot.set(0); // mRightPivot is set as reversed follower of mLeftPivot
        }
    }

    // commented out for safety's sake. same with reference to it in IntakeStowed
    // file

    @Override
    public void periodic() {
        SmartDashboard.putNumber( "ReportedPivotPosition", getPivotPos());
        SmartDashboard.putNumber( "PivotError",            desiredAngle - getPivotPos());
        SmartDashboard.putBoolean("leftDeploySwitch",      leftDeploySwitch.get());
        SmartDashboard.putBoolean("leftStowSwitch",        leftStowSwitch.get());
        SmartDashboard.putBoolean("rightDeploySwitch",     rightDeploySwitch.get());
        SmartDashboard.putBoolean("rightStowSwitch",       rightStowSwitch.get());

        if (leftDeploySwitch.get() || rightDeploySwitch.get()) {
            mLeftPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotDeployPos));
            mRightPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotDeployPos));
        }

        if (leftStowSwitch.get() || rightStowSwitch.get()) {
            mLeftPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotStowPos));
            mRightPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotStowPos));
        }


        // TODO Comments to explain the purpose and process used here
        // if (leftDeploySwitch.get()) {
        //     leftDeployPressed = true;
        // }
        // if (rightDeploySwitch.get()) {
        //     rightDeployPressed = true;
        // }

        // if (leftDeployPressed && !leftDeploySwitch.get()) {
        //     leftDeployPressed = false;
        //     mLeftPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotDeployPos));
        // }
        // if (rightDeployPressed && !rightDeploySwitch.get()) {
        //     rightDeployPressed = false;
        //     mRightPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotDeployPos));
        // }

        // if (leftStowSwitch.get() && !isCalibrated) {
        //     isCalibrated = true;
        //     mLeftPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotStowPos));
        // }

        // if (rightStowSwitch.get() && !isCalibrated) {
        //     isCalibrated = true;
        //     mRightPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotStowPos));
        // }

        // if (leftDeploySwitch.get() && !isCalibrated) {
        //     isCalibrated = true;
        //     mLeftPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotDeployPos));
        // }

        // if (rightDeploySwitch.get() && !isCalibrated) {
        //     isCalibrated = true;
        //     mRightPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotDeployPos));
        // }

        // TODO
        /*
        for (int i = 0; i < limitSwitches.length; i++) {
            if (limitSwitches[i].get()) {

            } else {

            }
        }
        for (int i = 0; i < limitSwitchFlags.length; i++) {
            if (limitSwitchFlags[i]) {

            } else {

            }

        }
        */
    }

}