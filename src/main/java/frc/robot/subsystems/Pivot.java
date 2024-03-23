package frc.robot.subsystems;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.IDConstants;
import frc.robot.CTREConfigs;

public class Pivot extends SubsystemBase {
    // motors
    public TalonFX mLeftPivot;
    public TalonFX mRightPivot;

    private final PositionVoltage anglePosition = new PositionVoltage(0);


    // limit switches
    /**Normally Open / True = safe*/ public DigitalInput leftDeploySwitch = new DigitalInput(IDConstants.Intooter.Pivot.leftOutSwitchID);
    /**Normally Open / True = safe*/ public DigitalInput leftStowSwitch = new DigitalInput(IDConstants.Intooter.Pivot.leftInSwitchID);
    /**Normally Open / True = safe*/ public DigitalInput rightDeploySwitch = new DigitalInput(IDConstants.Intooter.Pivot.rightOutSwitchID);
    /**Normally Open / True = safe*/ public DigitalInput rightStowSwitch = new DigitalInput(IDConstants.Intooter.Pivot.rightInSwitchID);
    // limits as checked during calibration, to account for encoder drift
    double intakeStowLimitPos;
    double intakeDeployLimitPos;

    double desiredAngle = -40;

    boolean deployPressed = false;
    boolean stowPressed = false;

    double voltageFromG, voltageFromPID, voltageToPivot, voltageFromResistance;
    // Pivot PDGR Control
    ArmFeedforward pivotGravityFeed = new ArmFeedforward(0.0, Constants.Intake.pivotKG, 0.0);
    ArmFeedforward pivotResistanceFeed = new ArmFeedforward(0.0, Constants.Intake.pivotKRes, 0.0);
    PIDController pivotPIDController = new PIDController(Constants.Intake.pivotKP, Constants.Intake.pivotKI, Constants.Intake.pivotKD, 0.02);

    private Swerve s_Swerve;
    private Pose2d pose;

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

        SmartDashboard.putNumber("COMP pivot position", 0);
        
        // Initialises motor controller objects and configures them
        mLeftPivot = new TalonFX(IDConstants.Intooter.Pivot.mLeftPivotID);
        mLeftPivot.getConfigurator().apply(CTREConfigs.leftPivotMotorFXConfig);
        mLeftPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotStowPos));

        mRightPivot = new TalonFX(IDConstants.Intooter.Pivot.mRightPivotID);
        mRightPivot.getConfigurator().apply(CTREConfigs.rightPivotMotorFXConfig);
        mRightPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotStowPos));
        mRightPivot.setControl(new Follower(mLeftPivot.getDeviceID(), true));
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
     * @param inputAngle The real-world angle to move the arm to in degrees.
     *                   Intake Postive,
     *                   Using limits switches.
     */
    public void moveArmToAngle(double inputAngle) {
        desiredAngle = inputAngle;
        System.out.println("Moving arm to angle" + inputAngle);
        SmartDashboard.putNumber("desiredAngle", desiredAngle);
        // double motorAngle = -(desiredAngle - Constants.Intake.pivotOffsetForZero);
        mLeftPivot.setControl(
                anglePosition.withPosition((inputAngle / 360))
        );
        mRightPivot.setControl(new Follower(mLeftPivot.getDeviceID(), true));
        // CTREConfigs already has Left and Right use opposite directions
    
        pivotPDGCycle(desiredAngle);
    }

    // .withLimitForwardMotion(rightDeploySwitch.get())
    // .withLimitForwardMotion(leftDeploySwitch.get())

    // .withLimitReverseMotion(rightStowSwitch.get())
    // .withLimitReverseMotion(leftStowSwitch.get())

    /**
     * Moves the arm to a set position, in degrees
     * @param inputAngle The real-world angle to move the arm to in degrees.
     *                   Intake Postive,
     *                   Using limits switches.
     * @author 5985
     */
    public void pivotPDGCycle(double inputAngle) 
    {
        desiredAngle = inputAngle;
        pivotPDGCycle();
    }

    private double pivotResCalculate(double angle) 
    {
        if (angle > 0) 
        {
            return Math.max(0, 2 * (angle - Constants.Intake.pivotResDeployThreshold));
        }
        else
        {
            return Math.min(0, 2 * (angle + Constants.Intake.pivotResStowThreshold));
        }   
    }


    /**
     * Moves the pivot to the desired angle, using PDG control. Respects limit switches 
     * @author 5985
     * @author Alec
     */
    public void pivotPDGCycle()
    {
        /*
        SmartDashboard.putNumber("Pivot PDG Position Degrees : ", getPivotPos() + 90);
        SmartDashboard.putNumber("Pivot PDG Position Radians : ", Math.toRadians(getPivotPos() + 90)/Math.PI);
        SmartDashboard.putNumber("Pivot PDG Cos : ", Math.cos(Math.toRadians(getPivotPos() + 90)));
        //SmartDashboard.putNumber("Pivot PDG b : ", 0d);

        

        SmartDashboard.putNumber("Grav Voltage Output", voltageFromG);
        SmartDashboard.putNumber("PID Voltage Output : ", voltageFromPID);
        SmartDashboard.putNumber("Voltage to Pivot", voltageToPivot);
        */
        
        // If either stow switch is pressed, and the desired angle or desired voltage is further in the stow direction than the current position, do not move
        if ((!leftStowSwitch.get() || !rightStowSwitch.get()) && desiredAngle <= getPivotPos()) 
        {
            voltageToPivot = 0;
            SmartDashboard.putString("Pivot PDG Status : ", "Stopped at Stow");
        }
        // If either deploy switch is pressed, and the desired angle or desired voltage is further in the deploy direction than the curren position, do not move
        else if ((!leftDeploySwitch.get() || !rightDeploySwitch.get()) && desiredAngle >= getPivotPos())
        {
            voltageToPivot = 0;
            SmartDashboard.putString("Pivot PDG Status : ", "Stopped at Deploy");
        }
        // If it is determined safe to move, move using the combined ArmFeedforward for gravity compensation and PID for angle control
        else
        {
            voltageFromG   = Constants.Intake.pivotKG * Math.cos((Math.toRadians(getPivotPos() + 90)));
            voltageFromResistance = Constants.Intake.pivotKRes * Math.cos(Math.toRadians(pivotResCalculate(getPivotPos())) + 90);
            voltageFromPID = pivotPIDController.calculate(getPivotPos(), desiredAngle);
            voltageToPivot = voltageFromG + voltageFromPID + voltageFromResistance;

            SmartDashboard.putString("Pivot PDG Status : ", "Running");
        }

        mLeftPivot.setVoltage(voltageToPivot);
    }

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
    public void setDesiredPostion(Double angle) 
    {
        System.out.println("Set Desired Position as" + angle);
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
        if ((speed < 0 && leftStowSwitch.get() && rightStowSwitch.get())
                || (speed > 0 && leftDeploySwitch.get() && rightDeploySwitch.get())) {
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

        if ((!leftDeploySwitch.get() || !rightDeploySwitch.get()) && !deployPressed) {
            //positionOffset = (Constants.Intake.pivotDeployPos)-Units.rotationsToDegrees(mLeftPivot.getPosition().getValueAsDouble());
            deployPressed = true;
            mLeftPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotDeployPos));
            mRightPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotDeployPos));
        }
        else if (leftDeploySwitch.get() && rightDeploySwitch.get()) {
            //positionOffset = (Constants.Intake.pivotDeployPos)-Units.rotationsToDegrees(mLeftPivot.getPosition().getValueAsDouble());
            deployPressed = false;
        }

        if ((!leftStowSwitch.get() || !rightStowSwitch.get()) && !stowPressed) {
            //positionOffset = (Constants.Intake.pivotDeployPos)-Units.rotationsToDegrees(mLeftPivot.getPosition().getValueAsDouble());
            stowPressed = true;
            mLeftPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotStowPos));
            mRightPivot.getConfigurator().setPosition(Units.degreesToRotations(Constants.Intake.pivotStowPos));
        }
        else if (leftStowSwitch.get() && rightStowSwitch.get()) {
            //positionOffset = (Constants.Intake.pivotDeployPos)-Units.rotationsToDegrees(mLeftPivot.getPosition().getValueAsDouble());
            stowPressed = false;
        }
        calculatedRequiredShooterAngle();
    }

    /**
     * Function to extrapolate and interpolate the values needed for the shooter
     * pivot based on the current reported distance to the target.
     * 
     * @return The value in degrees that the pivot needs to angle to to score in the
     *         speaker.
     */
    public double calculatedRequiredShooterAngle() {

        // double[] distances = Constants.distancesFromSpeaker; // distances in meters
        // double[] angles = Constants.anglesOfPivot; // shooter angles in degrees


        // double targetDistance = PhotonUtils.getDistanceToPose(pose, FieldConstants.translationToPose2d(FieldConstants.flipTranslation(FieldConstants.SPEAKER)));
        // SmartDashboard.putNumber("COMP Distance to the speaker", targetDistance);
        // // Ensure the target distance is within the range of the data
        // if (targetDistance < distances[0]) {
        //     // Extrapolate using the first two points
        //     return angles[0]
        //             + (angles[1] - angles[0]) * (targetDistance - distances[0]) / (distances[1] - distances[0]);
        // } else if (targetDistance > distances[distances.length - 1]) {
        //     // Extrapolate using the last twlo points
        //     int n = distances.length;
        //     return angles[n - 2] + (angles[n - 1] - angles[n - 2]) * (targetDistance - distances[n - 2])
        //             / (distances[n - 1] - distances[n - 2]);
        // }

        // // Perform linear interpolation
        // double angle = 0;
        // for (int i = 0; i < distances.length - 1; i++) {
        //     if (targetDistance >= distances[i] && targetDistance <= distances[i + 1]) {
        //         angle = angles[i] + (angles[i + 1] - angles[i]) * (targetDistance - distances[i])
        //                 / (distances[i + 1] - distances[i]);
        //         break;
        //     }
        // }

        double targetHeightOverShooter = 1.5;
        double shooterPivotOffset = 0.25;
        double targetAngle;
        double targetDistance = PhotonUtils.getDistanceToPose(pose, FieldConstants.translationToPose2d(FieldConstants.flipTranslation(FieldConstants.SPEAKER)));

        targetDistance += 0.17;

        targetAngle = Math.atan(targetHeightOverShooter/targetDistance);
        targetDistance += shooterPivotOffset * Math.tan(targetAngle);
        targetAngle = Math.toDegrees(Math.atan(targetHeightOverShooter/targetDistance));
        double error = Math.abs(desiredAngle - getPivotPos()) > 2 ? 0 : (desiredAngle - getPivotPos())/2;
        return  error + Math.min(Constants.Intake.pivotDeployPos, Math.max(Constants.Intake.pivotFrameClearPos, targetAngle));
        
        // return SmartDashboard.getNumber("COMP calculated shooter angle", angle);
        // return SmartDashboard.getNumber("COMP pivot position", 0);
        // return angle;




        // return Units.radiansToDegrees(Math.atan(
        // (2 - 0.425) / (PhotonUtils.getDistanceToPose(pose, new Pose2d(0.2, 5.6, new
        // Rotation2d(0, 0))))));
        // return 0;
    }

}