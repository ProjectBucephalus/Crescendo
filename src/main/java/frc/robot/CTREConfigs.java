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
        swerveCANcoderConfig.MagnetSensor.SensorDirection = SwerveConstants.cancoderInvert;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = SwerveConstants.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = SwerveConstants.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.angleCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.angleCurrentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = SwerveConstants.angleCurrentThresholdTime;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = SwerveConstants.angleKP;
        swerveAngleFXConfig.Slot0.kI = SwerveConstants.angleKI;
        swerveAngleFXConfig.Slot0.kD = SwerveConstants.angleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = SwerveConstants.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = SwerveConstants.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = SwerveConstants.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = SwerveConstants.driveKP;
        swerveDriveFXConfig.Slot0.kI = SwerveConstants.driveKI;
        swerveDriveFXConfig.Slot0.kD = SwerveConstants.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstants.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;




        /** Left Pivot Motor Config **/
        leftPivotMotorFXConfig.MotorOutput.Inverted = Constants.Intake.leftPivotMotorDirection;
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
        // leftPivotMotorFXConfig.Slot0.kP = Constants.Intake.pivotKP;
        // leftPivotMotorFXConfig.Slot0.kI = Constants.Intake.pivotKI;
        // leftPivotMotorFXConfig.Slot0.kD = Constants.Intake.pivotKD;

        /** Right Pivot Motor Config **/
        rightPivotMotorFXConfig.MotorOutput.Inverted = Constants.Intake.rightPivotMotorDirection;
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
        // rightPivotMotorFXConfig.Slot0.kP = Constants.Intake.pivotKP;
        // rightPivotMotorFXConfig.Slot0.kD = Constants.Intake.pivotKI;
        // rightPivotMotorFXConfig.Slot0.kI = Constants.Intake.pivotKD;

        rightPivotMotorFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Intake.openLoopRamp;
        rightPivotMotorFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Intake.openLoopRamp;
        
        leftPivotMotorFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Intake.openLoopRamp;
        leftPivotMotorFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Intake.openLoopRamp;
        

        
    }
}