package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public static TalonFXConfiguration leftArmMotorFXConfig = new TalonFXConfiguration();
    public static TalonFXConfiguration rightArmMotorFXConfig = new TalonFXConfiguration();

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




        /** Left Arm Motor Config **/
        leftArmMotorFXConfig.MotorOutput.Inverted = Constants.Intake.leftArmMotorInvert;
        leftArmMotorFXConfig.MotorOutput.NeutralMode = Constants.Intake.armMotorNeutralMode;

        /* Gear Ratio and Wrapping Config */
        leftArmMotorFXConfig.Feedback.SensorToMechanismRatio = Constants.Intake.armGearRatio;
        leftArmMotorFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        leftArmMotorFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Intake.armEnableCurrentLimit;
        leftArmMotorFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.armCurrentLimit;
        leftArmMotorFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Intake.armCurrentThreshold;
        leftArmMotorFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Intake.armCurrentThresholdTime;

        /* PID Config */
        leftArmMotorFXConfig.Slot0.kP = Constants.Intake.armKP;
        leftArmMotorFXConfig.Slot0.kI = Constants.Intake.armKI;
        leftArmMotorFXConfig.Slot0.kD = Constants.Intake.armKD;

        /** Right Arm Motor Config **/
        rightArmMotorFXConfig.MotorOutput.Inverted = Constants.Intake.rightArmMotorInvert;
        rightArmMotorFXConfig.MotorOutput.NeutralMode = Constants.Intake.armMotorNeutralMode;

        /* Gear Ratio and Wrapping Config */
        rightArmMotorFXConfig.Feedback.SensorToMechanismRatio = Constants.Intake.armGearRatio;
        rightArmMotorFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        rightArmMotorFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Intake.armEnableCurrentLimit;       
        rightArmMotorFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Intake.armCurrentThreshold;
        rightArmMotorFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Intake.armCurrentThresholdTime;
        rightArmMotorFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.armCurrentLimit;
 

        /* PID Config */
        rightArmMotorFXConfig.Slot0.kP = Constants.Intake.armKP;
        rightArmMotorFXConfig.Slot0.kD = Constants.Intake.armKI;
        rightArmMotorFXConfig.Slot0.kI = Constants.Intake.armKD;
        

        
    }
}