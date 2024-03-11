package frc.robot.subsystems;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    boolean deployPressed = false;
    boolean stowPressed = false;

    private Swerve s_Swerve;
    private Pose2d pose;

    // Sets the offset of the pivot so that if the limit switches are pressed 
    private double positionOffset = 0;

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

    public Pivot(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        // Prints values to Smart Dashboard
        SmartDashboard.putNumber("deployPosition", -95);
        SmartDashboard.putNumber("ampPosition", -60);

        // Initialises motor controller objects and configures them
        mLeftPivot = new TalonFX(Constants.Intake.mLeftPivotID);
        mLeftPivot.getConfigurator().apply(CTREConfigs.leftPivotMotorFXConfig);
        mLeftPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotStowPos));

        mRightPivot = new TalonFX(Constants.Intake.mRightPivotID);
        mRightPivot.getConfigurator().apply(CTREConfigs.rightPivotMotorFXConfig);
        mRightPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotStowPos));

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
                moveArmToAngle(calculatedRequiredShooterAngle());
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

    // So that the pivot angle adjusts based on how far away we are.
    public void updateSpeakerAngle() {
        moveArmToAngle(calculatedRequiredShooterAngle());
    }

    /**
     * moves the arm to a set position, in degrees
     * TODO Something is seriously wrong with the limit switch use and MUST be
     * resolved
     * TODO Angle calculations and conversion need to be recalibrated to use
     * reasonable real-world angles
     * 
     * @param inputAngle The real-world angle to move the arm to in degrees.
     *                   Intake Postive,
     *                   Using limits switches.
     */
    public void moveArmToAngle(double inputAngle) {
        desiredAngle = inputAngle;
        SmartDashboard.putNumber("desiredAngle", desiredAngle);
        // double motorAngle = -(desiredAngle - Constants.Intake.pivotOffsetForZero);
        mLeftPivot.setControl(
                anglePosition.withPosition((inputAngle / 360)-positionOffset)

        // .withLimitReverseMotion(leftStowSwitch.get())
        // .withLimitReverseMotion(rightStowSwitch.get())

        // .withLimitForwardMotion(leftDeploySwitch.get())
        // .withLimitForwardMotion(rightDeploySwitch.get())
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
     * 
     * @return Pivot position degrees
     */
    public double getPivotPos() {
        // motor.getPosition() returns rotations: convert to degrees and return average.
        return ((mRightPivot.getPosition().getValueAsDouble() * 360)
                + (mLeftPivot.getPosition().getValueAsDouble() * 360)) / 2;
        // return mRightPivot.getPosition().getValueAsDouble();
    }

    /**
     * Checks if the angle is within the acceptable tolerance
     * 
     * @return Boolean, true if current angle is acceptable
     */
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
    public void setArmMotorSpeeds(double speed) {
        if ((speed < 0 && !leftStowSwitch.get() && !rightStowSwitch.get())
                || (speed > 0 && !leftDeploySwitch.get() && !rightDeploySwitch.get())) {
            mLeftPivot.set(speed);
            // mRightPivot.set(speed); // mRightPivot is set as reversed follower of
            // mLeftPivot
            desiredAngle = getPivotPos();
        } else {
            mLeftPivot.set(0);
            // mRightPivot.set(0); // mRightPivot is set as reversed follower of mLeftPivot
        }
    }

    @Override
    public void periodic() {
        // Prints a bunch of values to the Smart Dashboard
        SmartDashboard.putNumber("ReportedPivotPosition", getPivotPos());
        SmartDashboard.putNumber("PivotError", desiredAngle - getPivotPos());
        SmartDashboard.putBoolean("leftDeploySwitch", leftDeploySwitch.get());
        SmartDashboard.putBoolean("leftStowSwitch", leftStowSwitch.get());
        SmartDashboard.putBoolean("rightDeploySwitch", rightDeploySwitch.get());
        SmartDashboard.putBoolean("rightStowSwitch", rightStowSwitch.get());

        pose = s_Swerve.getEstimatedPose();

        // If either outer limit switch is pressed, calibrates current motor positions
        // as their deployed positions

        // This causes command scheduler loop overruns for some reason
        if ((leftDeploySwitch.get() || rightDeploySwitch.get()) && !deployPressed) {
            //positionOffset = (Constants.Intake.pivotDeployPos)-Units.rotationsToDegrees(mLeftPivot.getPosition().getValueAsDouble());
            deployPressed = true;
            mLeftPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotDeployPos));
            mRightPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotDeployPos));
        }
        else if (!leftDeploySwitch.get() && !rightDeploySwitch.get()) {
            //positionOffset = (Constants.Intake.pivotDeployPos)-Units.rotationsToDegrees(mLeftPivot.getPosition().getValueAsDouble());
            deployPressed = false;
        }

        if ((leftStowSwitch.get() || rightStowSwitch.get()) && !stowPressed) {
            //positionOffset = (Constants.Intake.pivotDeployPos)-Units.rotationsToDegrees(mLeftPivot.getPosition().getValueAsDouble());
            stowPressed = true;
            mLeftPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotStowPos));
            mRightPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotStowPos));
        }
        else if (!leftStowSwitch.get() && !rightStowSwitch.get()) {
            //positionOffset = (Constants.Intake.pivotDeployPos)-Units.rotationsToDegrees(mLeftPivot.getPosition().getValueAsDouble());
            stowPressed = false;
        }

        // If either inner limit switch is pressed, calibrates current motor
        // positions as their stowed positions
        // if (leftStowSwitch.get() || rightStowSwitch.get()) {
            //positionOffset = (Constants.Intake.pivotStowPos)-Units.rotationsToDegrees(mLeftPivot.getPosition().getValueAsDouble());
            // mLeftPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotStowPos));
            // mRightPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotStowPos));
        // }

        // Not being used currently.

        // if (leftDeploySwitch.get()) {
        // leftDeployPressed = true;
        // }
        // if (rightDeploySwitch.get()) {
        // rightDeployPressed = true;
        // }

        // if (leftDeployPressed && !leftDeploySwitch.get()) {
        // leftDeployPressed = false;
        // mLeftPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotDeployPos));
        // }
        // if (rightDeployPressed && !rightDeploySwitch.get()) {
        // rightDeployPressed = false;
        // mRightPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotDeployPos));
        // }

        // if (leftStowSwitch.get() && !isCalibrated) {
        // isCalibrated = true;
        // mLeftPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotStowPos));
        // }

        // if (rightStowSwitch.get() && !isCalibrated) {
        // isCalibrated = true;
        // mRightPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotStowPos));
        // }

        // if (leftDeploySwitch.get() && !isCalibrated) {
        // isCalibrated = true;
        // mLeftPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotDeployPos));
        // }

        // if (rightDeploySwitch.get() && !isCalibrated) {
        // isCalibrated = true;
        // mRightPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotDeployPos));
        // }

        // TODO
        /*
         * for (int i = 0; i < limitSwitches.length; i++) {
         * if (limitSwitches[i].get()) {
         * 
         * } else {
         * 
         * }
         * }
         * for (int i = 0; i < limitSwitchFlags.length; i++) {
         * if (limitSwitchFlags[i]) {
         * 
         * } else {
         * 
         * }
         * 
         * }
         */
    }

    /**
     * Function to extrapolate and interpolate the values needed for the shooter
     * pivot based on the current reported distance to the target.
     * 
     * @return The value in degrees that the pivot needs to angle to to score in the
     *         speaker.
     */
    public double calculatedRequiredShooterAngle() {

        double[] distances = Constants.distancesFromSpeaker; // distances in meters
        double[] angles = Constants.anglesOfPivot; // shooter angles in degrees

        // SmartDashboard.putNumber("distance to target",
        // PhotonUtils.getDistanceToPose(pose, new Pose2d(0.2, 5.6, new Rotation2d(0,
        // 0))));

        double targetDistance = PhotonUtils.getDistanceToPose(pose, new Pose2d(0.2, 5.6, new Rotation2d(0, 0)));

        // Ensure the target distance is within the range of the data
        if (targetDistance < distances[0]) {
            // Extrapolate using the first two points
            return angles[0]
                    + (angles[1] - angles[0]) * (targetDistance - distances[0]) / (distances[1] - distances[0]);
        } else if (targetDistance > distances[distances.length - 1]) {
            // Extrapolate using the last two points
            int n = distances.length;
            return angles[n - 2] + (angles[n - 1] - angles[n - 2]) * (targetDistance - distances[n - 2])
                    / (distances[n - 1] - distances[n - 2]);
        }

        // Perform linear interpolation
        double angle = 0;
        for (int i = 0; i < distances.length - 1; i++) {
            if (targetDistance >= distances[i] && targetDistance <= distances[i + 1]) {
                angle = angles[i] + (angles[i + 1] - angles[i]) * (targetDistance - distances[i])
                        / (distances[i + 1] - distances[i]);
                break;
            }
        }
        return angle;

        // return Units.radiansToDegrees(Math.atan(
        // (2 - 0.425) / (PhotonUtils.getDistanceToPose(pose, new Pose2d(0.2, 5.6, new
        // Rotation2d(0, 0))))));
        // return 0;
    }

}