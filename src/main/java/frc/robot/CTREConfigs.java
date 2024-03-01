package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public static TalonFXConfiguration leftPivotMotorFXConfig = new TalonFXConfiguration();
    public static TalonFXConfiguration rightPivotMotorFXConfig = new TalonFXConfiguration();

    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs(){
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.angleCurrentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.angleCurrentThresholdTime;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;




        /** Left Pivot Motor Config **/
        leftPivotMotorFXConfig.MotorOutput.Inverted = Constants.Intake.leftPivotMotorInvert;
        leftPivotMotorFXConfig.MotorOutput.NeutralMode = Constants.Intake.pivotMotorNeutralMode;

        /* Gear Ratio and Wrapping Config */
        leftPivotMotorFXConfig.Feedback.SensorToMechanismRatio = Constants.Intake.pivotGearRatio;
        leftPivotMotorFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        leftPivotMotorFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Intake.pivotEnableCurrentLimit;
        leftPivotMotorFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.pivotCurrentThreshold;
        leftPivotMotorFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Intake.pivotCurrentThresholdTime;
        leftPivotMotorFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Intake.pivotCurrentLimit;

        /* PID Config */
        leftPivotMotorFXConfig.Slot0.kP = Constants.Intake.pivotKP;
        leftPivotMotorFXConfig.Slot0.kI = Constants.Intake.pivotKI;
        leftPivotMotorFXConfig.Slot0.kD = Constants.Intake.pivotKD;

        /** Right Pivot Motor Config **/
        rightPivotMotorFXConfig.MotorOutput.Inverted = Constants.Intake.rightPivotMotorInvert;
        rightPivotMotorFXConfig.MotorOutput.NeutralMode = Constants.Intake.pivotMotorNeutralMode;

        /* Gear Ratio and Wrapping Config */
        rightPivotMotorFXConfig.Feedback.SensorToMechanismRatio = Constants.Intake.pivotGearRatio;
        rightPivotMotorFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        rightPivotMotorFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Intake.pivotEnableCurrentLimit;       
        rightPivotMotorFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Intake.pivotCurrentThreshold;
        rightPivotMotorFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Intake.pivotCurrentThresholdTime;
        rightPivotMotorFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.pivotCurrentLimit;
 

        /* PID Config */
        rightPivotMotorFXConfig.Slot0.kP = Constants.Intake.pivotKP;
        rightPivotMotorFXConfig.Slot0.kD = Constants.Intake.pivotKI;
        rightPivotMotorFXConfig.Slot0.kI = Constants.Intake.pivotKD;

        rightPivotMotorFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Intake.openLoopRamp;
        rightPivotMotorFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Intake.openLoopRamp;
        
        leftPivotMotorFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Intake.openLoopRamp;
        leftPivotMotorFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Intake.openLoopRamp;
        

        
    }
}