package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.COTSTalonFXSwerveConstants.SDS.MK3.driveRatios;

public final class Constants {
    public static final double stickDeadband = 0.3;
    
    /* Shooter Constants */
    public static final double shooterAngleOffset = 15;
    public static final double horizontalShooterAngle = 20;
    public static final double mFlapMaxCurrent = 40;

    /* Image Tracking Constants */
    public static final double cameraPitchOffset = 26;
    public static final double speakerTagHeight = 144;
    public static final double cameraHeightOverGround = 23;
    public static final double targetHeightOverTag = 40;

    /* CAN IDs */
    public static final int pigeonID = 53;

    public static final class Vision {
        /* Names */
        public static final String leftCamName = "LeftCam";
        public static final String rightCamName = "RightCam";
        public static final String backCamName = "BackCam";

        /* Offsets */
        public static final Transform3d leftCamToRobot = new Transform3d( // Meters and Radians (roll, pitch, yaw)
            0,0,0.525,
            new Rotation3d(
            0,Units.degreesToRadians(45),Units.degreesToRadians(30)
            ));
        public static final Transform3d rightCamToRobot = new Transform3d( // Meters and Radians (roll, pitch, yaw)
            0,0,0.525,
            new Rotation3d(
            0,Units.degreesToRadians(45),Units.degreesToRadians(45)
            ));
        public static final Transform3d backCamToRobot = new Transform3d( // Meters and Radians (roll, pitch, yaw)
            0,0,0.525,
            new Rotation3d(
            0,0,0
            ));
        
    }

    public static final class Intake {
        public static final int mLeftPivotID = 13;
        public static final int mRightPivotID = 11;

        public static final int leftOutSwitchID = 1;
        public static final int leftInSwitchID = 0;
        public static final int rightInSwitchID = 2;
        public static final int rightOutSwitchID = 3;

        public static final int mFlapID = 14;

        public static final int mIntakeID = 12;

        public static final double FlapMaxCurrent = 40;

        /* Arm Ratios and Limis */
        public static final double pivotGearRatio = (28); 
        public static final double pivotGearMaxRange = 1.7; // Radians from stowed to intake pos  
        public static final double pivotAmpPos = 1.2;
        

        public static final NeutralModeValue pivotMotorNeutralMode = NeutralModeValue.Brake;
        public static final InvertedValue leftPivotMotorInvert = InvertedValue.Clockwise_Positive;
        public static final InvertedValue rightPivotMotorInvert = InvertedValue.CounterClockwise_Positive;

        public static double angleKP;
        public static double angleKI;
        public static double angleKD;
        public static final int pivotCurrentLimit = 38;
        public static final int pivotCurrentThreshold = 65;
        public static final double pivotCurrentThresholdTime = 0.1;
        public static final boolean pivotEnableCurrentLimit = false;

        public static double pivotKP = 20;
        public static double pivotKI = 0;
        public static double pivotKD = 0;
        
    }
    
    public static final class Shooter {
        public static final int mTopShooterID = 15;
        public static final int mBottomShooterID = 23;

        public static final double maxTopShooterSpeed = 0.8;
        public static final double maxBottomShooterSpeed = 0.8; // AMP TOP: 0.450000 bottom: 0.05

        public static final double shooterIdleSpeed = 0.5;
    }

    public static final class Climber {
        public static final int mLeftClimbID = 17;
        public static final int mRightClimbID = 14;
        public static final double maxRevolutions = 320; //3.2 with gear ratio
    }

    public static final class Swerve {
        
        public static final boolean invertGyro = false;

        public static final double brakeIntensity = 0.15; // 0.25 -> Trigger fully pressed -> quarter speed.

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.48;
        public static final double wheelBase = 0.48;
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 27;
        public static final int angleCurrentThreshold = 45;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 38;
        public static final int driveCurrentThreshold = 65;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;


        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.1; 
        public static final double driveKI = 0.03;
        public static final double driveKD = 0.04;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); 
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 8.0; //2.5 TODO it was 8
        /** Radians per Second */
        public static final double maxAngularVelocity = 15.0; //5.0?? it was 15

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(90);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(270);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(180);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }
    

    public static final class AutoConstants { //TODO
        public static final double kMaxSpeedMetersPerSecond = 4;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI; // was pi?
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI; // was pi?
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
