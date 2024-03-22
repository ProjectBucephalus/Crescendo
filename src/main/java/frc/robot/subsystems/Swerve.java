package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.IDConstants;
import frc.robot.RobotContainer;
import frc.robot.SwerveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Swerve extends SubsystemBase 
{

    // Creates a poseEstimator object, which stores and estimates the robot's field relative pose
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator poseEstimator;
    
    // Creates photonPoseEstimator objects for both cameras, which estimate the camera's pose relative to the field
    public PhotonPoseEstimator photonPoseEstimatorFront;
    public PhotonPoseEstimator photonPoseEstimatorBack;
    
    // Creates objects representing both cameras
    public PhotonCamera frontCam = new PhotonCamera(Constants.Vision.frontCamName);
    public PhotonCamera backCam = new PhotonCamera(Constants.Vision.backCamName);

    // Creates an object representing the field in 2d
    private final Field2d m_field = new Field2d();
    private final Field2d m_Notesfield = new Field2d();

    // set to true initially so that if we manually set the angle and dont use any auto functions it will still shoot
    private boolean alignedToTarget = true;

    public boolean usingVisionAlignment = false;

    final AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); 

    // Creates config object for the path follower commands
    private final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
            new PIDConstants(Constants.AutoConstants.kPXController),
            new PIDConstants(Constants.AutoConstants.kPYController), Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            SwerveConstants.trackWidth, new ReplanningConfig());

    /** List of swerve module motors */
    public SwerveModule[] mSwerveMods;
    
    /** Robot's gyro [X Right, Y Forward, Z Up] */
    public Pigeon2 gyro;
    
    /** Robot's field relative position */
    public Pose2d pose;

    private Optional<EstimatedRobotPose> visionEstimatedPoseFront, visionEstimatedPoseBack;
    private EstimatedRobotPose estimatedRobotPose;

//         // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
//     private final MutableMeasure<Voltage> m_appliedVoltage = mutable(BaseUnits.Voltage.of(0));
//     // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
//     private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
//     // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
//     private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));


//     // Create a new SysId routine for characterizing the drive.
//   private final SysIdRoutine m_sysIdRoutine =
//       new SysIdRoutine(
//           // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
//           new SysIdRoutine.Config(),
//           new SysIdRoutine.Mechanism(
//               // Tell SysId how to plumb the driving voltage to the motors.
//               (Measure<Voltage> volts) -> {
//                 m_leftMotor.setVoltage(volts.in(Volts));
//                 m_rightMotor.setVoltage(volts.in(Volts));
//               },
//               // Tell SysId how to record a frame of data for each motor on the mechanism being
//               // characterized.
//               log -> {
//                 // Record a frame for the left motors.  Since these share an encoder, we consider
//                 // the entire group to be one motor.
//                 log.motor("drive-left")
//                     .voltage(
//                         m_appliedVoltage.mut_replace(
//                             m_leftMotor.get() * RobotController.getBatteryVoltage(), Volts))
//                     .linearPosition(m_distance.mut_replace(m_leftEncoder.getDistance(), Meters))
//                     .linearVelocity(
//                         m_velocity.mut_replace(m_leftEncoder.getRate(), MetersPerSecond));
//                 // Record a frame for the right motors.  Since these share an encoder, we consider
//                 // the entire group to be one motor.
//                 log.motor("drive-right")
//                     .voltage(
//                         m_appliedVoltage.mut_replace(
//                             m_rightMotor.get() * RobotController.getBatteryVoltage(), Volts))
//                     .linearPosition(m_distance.mut_replace(m_rightEncoder.getDistance(), Meters))
//                     .linearVelocity(
//                         m_velocity.mut_replace(m_rightEncoder.getRate(), MetersPerSecond));
//               },
//               // Tell SysId to make generated commands require this subsystem, suffix test state in
//               // WPILog with this subsystem's name ("drive")
//               this));

    public Swerve() {
        // Define and initialise gyro, as well as applying config
        gyro = new Pigeon2(IDConstants.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        // Define and initialise list of swerve modules
        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, SwerveConstants.Mod0.constants),
                new SwerveModule(1, SwerveConstants.Mod1.constants),
                new SwerveModule(2, SwerveConstants.Mod2.constants),
                new SwerveModule(3, SwerveConstants.Mod3.constants)
        };

        // Define and initialise pose estimator
        swerveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions());
        poseEstimator = new SwerveDrivePoseEstimator(
                SwerveConstants.swerveKinematics,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(),
                Constants.Vision.STATE_STANDARD_DEVIATIONS,
                Constants.Vision.VISION_MEASUREMENT_STANDARD_DEVIATIONS);

        // Define and initialise PhotonPoseEstimators
        photonPoseEstimatorFront = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCam,
                Constants.Vision.frontCamToRobot);
        photonPoseEstimatorBack = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCam,
                Constants.Vision.backCamToRobot);
        photonPoseEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonPoseEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Configures the AutoBuilder
        AutoBuilder.configureHolonomic
        (
                this::getEstimatedPose, // Robot pose supplier
                this::resetEstimatedOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(10, 0.3, 0.3), // Translation PID constants
                        new PIDConstants(10, 0.3, 0.3), // Rotation PID constants
                        100, // Max module speed, in m/s
                        0.34, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

    }

    /**
     * 364 Magic
     * 
     * @param translation
     * @param rotation
     * @param fieldRelative
     * @param isOpenLoop
     * @author 364
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, double brakeVal) 
    {
        if (!usingVisionAlignment) 
        {
            SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates
            (
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds
                (
                    translation.getX(),
                    translation.getY(),
                    rotation,
                    getHeading()
                )
                : new ChassisSpeeds
                (
                    translation.getX(),
                    translation.getY(),
                    rotation
                )
            );
            
            SwerveDriveKinematics.desaturateWheelSpeeds
                (swerveModuleStates, SwerveConstants.maxSpeed * (map(brakeVal, 0, 1, SwerveConstants.brakeIntensity, 1)));
            
            for (SwerveModule mod : mSwerveMods) 
            {
                mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
            }
        }
    }

    //     /**
    //  * Returns a command that will execute a quasistatic test in the given direction.
    //  *
    //  * @param direction The direction (forward or reverse) to run the test in
    //  */
    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutine.quasistatic(direction);
    // }

    // /**
    //  * Returns a command that will execute a dynamic test in the given direction.
    //  *
    //  * @param direction The direction (forward or reverse) to run the test in
    //  */
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutine.dynamic(direction);
    // }

    /**
     * Swerve magic
     * 
     * @param translation
     * @param rotation
     * @param isOpenLoop
     * @param brakeVal
     * @author 5985
     */
    public void visionDrive(Translation2d translation, double rotation, boolean isOpenLoop, double brakeVal) 
    {
        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates
        (
            ChassisSpeeds.fromFieldRelativeSpeeds
            (
                translation.getX(),
                translation.getY(),
                rotation,
                getHeading()
            )
        );
        SwerveDriveKinematics.desaturateWheelSpeeds
            (swerveModuleStates, SwerveConstants.maxSpeed * (map(brakeVal, 0, 1, SwerveConstants.brakeIntensity, 1)));
        for (SwerveModule mod : mSwerveMods) 
        {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /**
     * Swerve magic
     * 
     * @param xSpeed
     * @param ySpeed
     * @param rot
     * @param fieldRelative
     * @author 5985
     */
    public void ChoreoDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) 
    {
        var swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates
        (
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);
        for (SwerveModule mod : mSwerveMods) 
        {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }

    /**
     * Used by SwerveControllerCommand in Auto
     * 
     * @param desiredStates A list of the states for each module to be set to
     * @author 364
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) 
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) 
        {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /**
     * Gets all of the swerve module states
     * @return A list containing the state of each swerve module
     * @author 364
     */
    public SwerveModuleState[] getModuleStates() 
    {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) 
        {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * Gets all of the swerve module positions
     * @return A list containing the position of each swerve module
     * @author 364
     */
    public SwerveModulePosition[] getModulePositions() 
    {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) 
        {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Gets the current 2d pose of the robot
     * @return The 2d pose of the robot
     * @author 364
     */
    public Pose2d getPose() 
    {
        return swerveOdometry.getPoseMeters();
    }

    /**
     * Gets whether the robot is aligned to the target
     * @return Whether the robot is aligned to the target. True or false.
     */
    public boolean getWithinRequiredHeading() 
    {
        return alignedToTarget;
    }
   
    /**
     * Sets whether the robot is aligned to the target
     * @param aligned The value for it to be set to. True or false
     * 
     */
    public void setWithinRequiredHeading(boolean aligned) 
    {
        this.alignedToTarget = aligned;
    }

    /**
     * Gets the current rotation of the robot
     * @return The current 2d rotation of the robot
     * @author 364
     */
    public Rotation2d getHeading() 
    {
        return getPose().getRotation();
    }

    /**
     * Resets the heading so that the current heading is zero
     * @author 364
     */
    public void zeroHeading() 
    {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    /**
     * Gets the current yaw angle reported by the gyro
     * @return The gyro's yaw angle, as a rotation2d object
     * @author 364
     */
    public Rotation2d getGyroYaw() 
    {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    /**
     * Gets the current overall field relative rotation reported by the gyro
     * @return The field relative 2D rotation, from the gyro
     * @author 5985
     * @author Aidan
     */
    public Rotation2d getGyro() 
    {
        return gyro.getRotation2d();
    }

    /**
     * Rotates all swerve modules back to their absolute zero positions
     * @author 364
     */
    public void resetModulesToAbsolute() 
    {
        for (SwerveModule mod : mSwerveMods) 
        {
            mod.resetToAbsolute();
        }
    }

    /**
     * 
     */
    public static double map(double valueCoord1,
            double startCoord1, double endCoord1,
            double startCoord2, double endCoord2) 
    {
        double R = (endCoord2 - startCoord2) / (endCoord1 - startCoord1);
        double y = startCoord2 + (valueCoord1 * R) + R;
        return (y);
    }

    /**
     * Sets the estimated position to a specified value
     * @param pose The value to set the estimated position to
     * @author 5985
     * @author Aidan
     */
    public void resetEstimatedOdometry(Pose2d pose) 
    {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    /**
     * Returns the current esimated position
     * @return The current estimated 2d position
     * @author 5985
     * @author Aidan
     */
    public Pose2d getEstimatedPose() 
    {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Set to true to ignore roatational values from the controller and use values
     * from vision drive or driveRobotRelative
     * 
     * @param newVal Set to true to ignore rotational inputs and use roation from
     *               driveRobotRelative(). Set to false to revert.
     */
    public void setVisionAlignmentBool(boolean newVal) 
    {
        usingVisionAlignment = newVal;
    }

    /**
     * Returns true if we are currently auto aligning. 
     * 
     * @return if we are auto aligning.
     */
    public boolean getVisionAlignmentBool() 
    {
        return usingVisionAlignment;
    }


    /**
     * Gets the current speed of the robot
     * @return A ChassisSpeeds object representing the current speed of the robot chassis
     * @author Unknown
     */
    public ChassisSpeeds getRobotRelativeSpeeds() 
    {
        return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Drives the robot at a set speed
     * 
     * @param robotRelativeSpeeds A ChassisSpeeds object representing the desired speed of the robot chassis
     * @author 5985
     * @author Unknown
     */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    /**
     * Uses the number of targets found and the closest of those targets to calculate the confidence of the estimation
     * @param estimation The estimated robot pose to find the confidence of
     * @return A matrix representing the confidence of it's pose estimation
     * @author Unknown online repository
     */
    private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
        double smallestDistance = Double.POSITIVE_INFINITY;
        for (var target : estimation.targetsUsed) {
            var t3d = target.getBestCameraToTarget();
            var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
            if (distance < smallestDistance)
                smallestDistance = distance;
        }
        double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
                ? 1 : Math.max
                (
                        1,
                        (estimation.targetsUsed.get(0).getPoseAmbiguity()
                                + Constants.Vision.POSE_AMBIGUITY_SHIFTER)
                                * Constants.Vision.POSE_AMBIGUITY_MULTIPLIER);
        double confidenceMultiplier = Math.max(
                1,
                (Math.max(
                        1,
                        Math.max(0, smallestDistance - Constants.Vision.NOISY_DISTANCE_METERS)
                                * Constants.Vision.DISTANCE_WEIGHT)
                        * poseAmbiguityFactor)
                        / (1
                                + ((estimation.targetsUsed.size() - 1) * Constants.Vision.TAG_PRESENCE_WEIGHT)));

        return Constants.Vision.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
    }

    /**
     * Function to extrapolate and interpolate the values needed for the shooter
     * pivot based on the current reported distance to the target.
     * 
     * @return The value in degrees that the pivot needs to angle to to score in the
     *         speaker.
     */
    

    /**
     * Takes a path from pathplanner, and turns it into a command to follow that path
     * 
     * @param path The pathplanner path to follow
     * @return The command for following that path
     * @author Unknown
     */
    public Command makePathFollowingCommand(PathPlannerPath path) 
    {
        // return new FollowPathHolonomic
        // (path, this::getEstimatedPose, this::getRobotRelativeSpeeds, this::driveRobotRelative,
        //         PATH_FOLLOWER_CONFIG,
        //         () -> {
        //             // Boolean supplier that controls when the path will be mirrored for the red
        //             // alliance
        //             // This will flip the path being followed to the red side of the field.
        //             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //             var alliance = DriverStation.getAlliance();
        //             if (alliance.isPresent()) {
        //                 return alliance.get() == DriverStation.Alliance.Red;
        //             }
        //             return false;
        //         }, this
        // );

        return AutoBuilder.followPath(path);
    }

    /**
     * Lock wheels in x position to resist pushing
     * @author 5985
     */
    public void lockWheels() {
    //     double lockRadians = Math.toRadians(45);
    //     for (SwerveModule mod : mSwerveMods) {
    //         mod.setDesiredState(new SwerveModuleState(), usingVisionAlignment);
    //     }
    //     double lockRadians = Math.toRadians(45);
    //     m_swerveModules[0].set(0.0, lockRadians);
    //     m_swerveModules[1].set(0.0, -lockRadians);
    //     m_swerveModules[2].set(0.0, -lockRadians);
    //     m_swerveModules[3].set(0.0, lockRadians);
    }

    

    @Override
    /**
     * TODO docs
     */
    public void periodic() {

        swerveOdometry.update(getGyroYaw(), getModulePositions());

        m_field.setRobotPose(getEstimatedPose());

        //final Optional<EstimatedRobotPose> 
        visionEstimatedPoseFront = photonPoseEstimatorFront.update();
        if (visionEstimatedPoseFront.isPresent()) {
            SmartDashboard.putBoolean("Using Vision", true);
            //final EstimatedRobotPose 
            estimatedRobotPose = visionEstimatedPoseFront.get();
            poseEstimator.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds,
                    confidenceCalculator(estimatedRobotPose));
        } else {
            SmartDashboard.putBoolean("Using Vision", false);
        }

        //final Optional<EstimatedRobotPose> 
        visionEstimatedPoseBack = photonPoseEstimatorBack.update();
        if (visionEstimatedPoseBack.isPresent()) {
            SmartDashboard.putBoolean("Using Vision", true);
            //final EstimatedRobotPose 
            estimatedRobotPose = visionEstimatedPoseBack.get();
            poseEstimator.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds,
                    confidenceCalculator(estimatedRobotPose));
        } else {
            SmartDashboard.putBoolean("Using Vision", false);
        }

        poseEstimator.update(getGyro(), getModulePositions());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            ///SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        SmartDashboard.putNumber("Pose X (Estimated)", getEstimatedPose().getX());
        SmartDashboard.putNumber("Pose Y (Estimated)", getEstimatedPose().getY());
        SmartDashboard.putNumber("Rotaton (Estimated)", getEstimatedPose().getRotation().getDegrees());

        SmartDashboard.putBoolean("usingVisionAlignment", usingVisionAlignment);
        SmartDashboard.putBoolean("getWithinRequiredHeading", getWithinRequiredHeading());

        SmartDashboard.putNumber("distance to target", PhotonUtils.getDistanceToPose(getEstimatedPose(), FieldConstants.flipPose(FieldConstants.translationToPose2d(FieldConstants.SPEAKER))));
        

        // Do this in either robot or subsystem init
        SmartDashboard.putData("Field", m_field);

    }

    public void simulationPeriodic() {
        // resetEstimatedOdometry();
    }

}