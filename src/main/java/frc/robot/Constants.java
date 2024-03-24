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
                -0.18, -0.21, 0.455,
                new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(180))
        ); // Meters and Radians (roll, pitch, yaw)

        public static final Transform3d frontCamToRobot = new Transform3d
        ( 
                0, 0, 0,
                new Rotation3d(0,Units.degreesToRadians(-20), Units.degreesToRadians(0))
        ); // Meters and Radians (roll, pitch, yaw)

        public static final Transform3d noteCamToRobot = new Transform3d
        (
                0, 0, 0,
                new Rotation3d(0, Units.degreesToRadians(32.5), Units.degreesToRadians(180))
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

    }

    public static final class Intake 
    {
        /* Arm Ratios and Limis */
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

        public static final double ANGLE_TOLERANCE_DEGREE = 1;

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
        public static final InvertedValue leftPivotMotorInvert = InvertedValue.Clockwise_Positive;
        public static final InvertedValue rightPivotMotorInvert = InvertedValue.CounterClockwise_Positive;

        public static double angleKP;
        public static double angleKI;
        public static double angleKD;

        public static final double pivotKP = 0; //0.3
        public static final double pivotKI = 0;
        public static final double pivotKD = 0; //0.03
        public static final double pivotKG = 0.5; //0.5
        public static final double pivotKRes = -0.25; // -0.25

        /** Degrees to Stow where Resistance begins*/
        public static final double pivotResStowThreshold = -50; // set to <= -40 for no Resistance in Stow direction
        /** Degrees to Deploy where Resistance begins*/
        public static final double pivotResDeployThreshold = 45; // set to >= 60 for no Resistance in Deploy direction

        public static final int pivotCurrentLimit = 38;
        public static final int pivotCurrentThreshold = 65;
        public static final double pivotCurrentThresholdTime = 0.1;
        public static final boolean pivotEnableCurrentLimit = false;
        public static final double pivotManualGain = 0.25;

        public static final double openLoopRamp = 1.3;


        //public static double pivotKP = 45; // 100
        //public static double pivotKI = 5; // 20
        //public static double pivotKD = 1; // 2 ssh

    }

    public static final class Shooter 
    {
        public static final double runningTopShooterSpeed = 1;
        public static final double runningBottomShooterSpeed = 1; // AMP TOP: 0.450000 bottom: 0.05

        public static final double shooterIdleSpeed = 0.3;

        public static final double ShooterAcceptableVelocity = 75;

        // public static final double horizontalShooterAngle = 20;

        public static final double openLoopRamp = 0;
    }

    public static final class Climber 
    {
        public static final double climbUpSpeed = 1;
        public static final double climbDownSpeed = -1;
        
        
        public static final double maxExtensionSpoolRotations = 2.6;
        public static final double motorToSpoolGearRatio = 100;
        public static final double maxRevolutions = maxExtensionSpoolRotations * motorToSpoolGearRatio;
        public static final double climberDownPos = 0;
        public static final double climberUpPos = maxExtensionSpoolRotations * motorToSpoolGearRatio;

    }

    public static final class AutoConstants 
    { // TODO
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI; // was pi?
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI; // was pi?

        public static final double kPXController = 1; // 2.9
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
