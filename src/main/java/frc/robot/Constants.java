package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants 
{
    public static boolean useVision = true;

    public static final double stickDeadband = 0.3;

    public static final double[] distancesFromSpeaker = { 1.8,    2, 2.5,  3, 3.5,  4, 5, 6 }; // distances in meters
    // TODO Values to calibrate: 3.5
    public static final double[] anglesOfPivot =        {  39, 29.5,  28, 23,  22, 20, 20,20 }; // shooter angles in degrees

    public static final class Vision 
    {
        /* Names */
        public static final String frontCamName = "DriveBaseCam";
        public static final String backCamName = "BackCam";
        public static final String noteCameraName = "IntakeCam";

        /* Offsets */
        // relative position of the camera on the robot to the robot center
        // pitch is the Y angle, and it is positive down
        public static final Transform3d backCamToRobot = new Transform3d
        ( 
                //0.18, -0.21, 0.455,
                -0.18, -0.18, 0.44,
                new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(180))
        ); // Meters and Radians (roll, pitch, yaw)

        public static final Transform3d frontCamToRobot = new Transform3d
        ( 
                0.28, -0.17, 0.19,
                new Rotation3d(Units.degreesToRadians(90),Units.degreesToRadians(38), Units.degreesToRadians(0))
        ); // Meters and Radians (roll, pitch, yaw)

        public static final Transform3d noteCamToRobot = new Transform3d
        (
                0.32, 0, 0.45,
                new Rotation3d(0, Units.degreesToRadians(20), Units.degreesToRadians(0))
        ); // Meters and Radians (roll, pitch, yaw)


        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
        public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
        public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
        public static final double NOISY_DISTANCE_METERS = 2.5;
        public static final double DISTANCE_WEIGHT = 7;
        public static final int TAG_PRESENCE_WEIGHT = 10;

        public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = MatBuilder.fill
        (Nat.N3(), Nat.N1(),
                // if these numbers are less than one, multiplying will do bad things
                1, // x
                1, // y
                1 * Math.PI // theta
        );

        /**
         * Standard deviations of the vision measurements. Increase these numbers to
         * trust global measurements from vision
         * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
         * radians.
         */
        public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = MatBuilder.fill(Nat.N3(), Nat.N1(), .1, .1, 1);

        public static final double noteTurnScalarGain = 10;
        public static final double noteTurnPowerGain = 3;
    }

    public static final class Intake 
    {
        /* Arm Ratios and Limits */
        public static final double planetaryRingTeeth = 72;
        public static final double planetarySunTeeth = 18;
        public static final double planetaryPlanetTeeth = 18;
        public static final double planetaryRatio = (planetaryRingTeeth / planetarySunTeeth) + 1;
        public static final double gear1In = 20;
        public static final double gear1Out = 76;
        public static final double pivotGearIn = 10;
        public static final double pivotGearOut = 40;
        public static final double pivotGearRatio = planetaryRatio * (gear1Out / gear1In)
                * (pivotGearOut / pivotGearIn);

        /** Acceptable angle for the pivot to be off by, in degrees */
        public static final double pivotAngleTolerance = 1;

        /** Degrees - Difference between pivot mechanism 0 and real-world 0 */
        public static final double pivotOffsetForZero = -37;
        /** Degrees - Real-world angle for deployed/intake position */
        public static final double pivotDeployPos = 60;
        /** Degrees - Real-world angle for stowed position */
        public static final double pivotStowPos = -40;
        /** Degrees - Real-world angle for shooting to Amp */
        public static final double pivotAmpPos = 47;
        /** Degrees - Real-world angle for shooting to Stage Trap */
        public static final double pivotTrapPos = pivotStowPos;
        /** Degrees - Real-world angle for shooter to clear frame */
        public static final double pivotFrameClearPos = 15;
        /** Degrees - Real-world angle for default shooter position */
        public static final double pivotDefaultShootPos = 0;

        public static final NeutralModeValue pivotMotorNeutralMode = NeutralModeValue.Brake;
        public static final InvertedValue leftPivotMotorDirection = InvertedValue.Clockwise_Positive;
        public static final InvertedValue rightPivotMotorDirection = InvertedValue.CounterClockwise_Positive;

        /* Gain values */
        public static final double pivotKP = 0.2; //0.3
        public static final double pivotKI = 0;
        public static final double pivotKD = 0.0; //0.03
        public static final double pivotKG = 0.85; //0.5
        public static final double pivotKRes = -0.25; // -0.25
        public static final double pivotDampingGain = 0.2;
        public static final double pivotManualGain = 0.25;

        /* Thresholds for damping to take effect */
        public static final double pivotDeployDampingThreshold = 20;
        public static final double pivotStowDampingThreshold = -5;
        /** Acceptable rotations per second of the mechanism towards endstops, manual control reaches 0.4 */
        public static final double pivotDampingSpeed = 0.4;
        
        /** Seconds to ramp power to new value */
        public static final double openLoopRamp = 0.1; 

        /** Degrees to Stow where Resistance begins*/
        public static final double pivotResStowThreshold = -50; // set to <= -40 for no Resistance in Stow direction
        /** Degrees to Deploy where Resistance begins*/
        public static final double pivotResDeployThreshold = 45; // set to >= 60 for no Resistance in Deploy direction

        /* Current limit values */
        public static final int pivotCurrentLimit = 38;
        public static final int pivotCurrentThreshold = 65;
        public static final double pivotCurrentThresholdTime = 0.1;
        public static final boolean pivotEnableCurrentLimit = false;
        
        /* Intake Speeds */
        public static final double intakeSpeedShoot = 1;
        public static final double intakeSpeedIn = 0.5;
        public static final double intakeSpeedOut = -0.35;
        public static final double intakeSpeedInWithLimit = 0.75;

        /* Indexer Speeds */
        public static final double indexSpeedIn = 0.25;
        public static final double indexSpeedOut = -0.35;
        public static final double indexSpeedInWithLimit = -0.4;
        public static final double indexSpeedShoot = 1;

    }

    public static final class Shooter 
    {
        /* Shooter speeds */
        public static final double runningTopShooterSpeed = 0.9;
        public static final double runningBottomShooterSpeed = 0.9; // AMP TOP: 0.450000 bottom: 0.05
        public static final double shooterIdleSpeed = 0.5;
        public static final double shooterEjectSpeed = -0.5;

        /** Acceptable velocity for the shooter to be off by, in rotations per second (?) */
        public static final double shooterVelocityTolerance = 80;

        // public static final double horizontalShooterAngle = 20;

        /** Seconds to ramp power to new value */
        public static final double openLoopRamp = 0;
        
        public static final double gravity = 9.8;
        
        /** Effective velocity of the ring coming out of the shooter, in meters per second */
        public static final double shooterVelocity = 19;

        public static final double verticalAccelerationConstant = gravity / (2 * Math.pow(shooterVelocity,2));

        /** Metres of target point over shooter exit */
        public static final double targetHeightOverShooter = 1.4;
        /** Metres of target point in front of tag */
        public static final double targetDistanceOffset = 0.23;
        /** Metres of shooter exit over pivot axis */
        public static final double shooterPivotOffsetUp = 0.25;
        /** Metres of pivot behind robot centre */
        public static final double shooterPivotOffsetBack = -0.17;

        /** Maximum viable shot distance, Metres, past this lob notes to subwoofer for others to take */
        public static final double maxShootDistance = 8;
        /** Shooter Angle for hard-coded lob */
        public static final double halfCourtAngle = 30;
    }

    public static final class Climber 
    {   
        /* Climber motion speeds */
        public static final double climbUpSpeed = 1;
        public static final double climbDownSpeed = -1;
        
        /* Climber real world values */
        public static final double maxExtensionSpoolRotations = 2.6;
        public static final double motorToSpoolGearRatio = 100;
        public static final double maxRevolutions = maxExtensionSpoolRotations * motorToSpoolGearRatio;
        
        /* Climber positions */
        public static final double climberDownPos = 0;
        public static final double climberUpPos = maxExtensionSpoolRotations * motorToSpoolGearRatio;

    }

    public static final class AutoConstants 
    { 
        /** Max drivebase speed, in meters per second */
        public static final double kMaxSpeedMetersPerSecond = 2;
        /** Max drivebase acceleration, in meters per second per second */
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        /** Max drivebase rotational speed, in radians per second */
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI; // was pi?
        /** Max drivebase rotational acceleration, in radians per second per second */
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI; // was pi?

        public static final double kPXController = 1; // 2.9
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
    }
}
