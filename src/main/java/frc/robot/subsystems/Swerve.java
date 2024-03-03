package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

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
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    // set to true initially so that if we manually set the angle and dont use any auto functions it will still shoot
    private boolean alignedToTarget = true;

    public boolean usingVisionAlignment = false;

    final AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); 
    // see docs for how to do this better and set origin for red alliance

    // Creates config object for the path follower commands
    private final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
            new PIDConstants(Constants.AutoConstants.kPXController),
            new PIDConstants(Constants.AutoConstants.kPYController), Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.Swerve.trackWidth, new ReplanningConfig());

    /** List of swerve module motors */
    public SwerveModule[] mSwerveMods;
    
    /** Robot's gyro [X Right, Y Forward, Z Up] */
    public Pigeon2 gyro;
    
    /** Robot's field relative position */
    public Pose2d pose;

    private Optional<EstimatedRobotPose> visionEstimatedPoseFront, visionEstimatedPoseBack;
    private EstimatedRobotPose estimatedRobotPose;

    public Swerve() {
        // Define and initialise gyro, as well as applying config
        gyro = new Pigeon2(Constants.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        // Define and initialise list of swerve modules
        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        // Define and initialise pose estimator
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
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
            SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates
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
                (swerveModuleStates, Constants.Swerve.maxSpeed * (map(brakeVal, 0, 1, Constants.Swerve.brakeIntensity, 1)));
            
            for (SwerveModule mod : mSwerveMods) 
            {
                mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
            }
        }
    }

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
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates
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
            (swerveModuleStates, Constants.Swerve.maxSpeed * (map(brakeVal, 0, 1, Constants.Swerve.brakeIntensity, 1)));
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
        var swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates
        (
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
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
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

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
     * from vision
     * 
     * @param newVal Set to true to ignore rotational inputs and use roation from
     *               driveRobotRelative(). Set to false to revert.
     */
    public void setVisionAlignmentBool(boolean newVal) 
    {
        usingVisionAlignment = newVal;
    }

    /**
     * Gets the current speed of the robot
     * @return A ChassisSpeeds object representing the current speed of the robot chassis
     * @author Unknown
     */
    public ChassisSpeeds getRobotRelativeSpeeds() 
    {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
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

        SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    /**
     * Uses the number of targets found and the closest of those targets to calculate the confidence of the estimation
     * @param estimation The estimated robot pose to find the confidence of
     * @return A matrix representing the confidence of it's pose estimation
     * @author Unknown online repository
     */
    private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) 
    {
        double smallestDistance = Double.POSITIVE_INFINITY;
        
        for (var target : estimation.targetsUsed) 
        {
            var t3d = target.getBestCameraToTarget();
            var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
            if (distance < smallestDistance)
                smallestDistance = distance;
        }

        double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
                ? 1 : Math.max
                (
                    1,
                    (estimation.targetsUsed.get(0).getPoseAmbiguity() + Constants.Vision.POSE_AMBIGUITY_SHIFTER)
                        * Constants.Vision.POSE_AMBIGUITY_MULTIPLIER
                );
        
                double confidenceMultiplier = Math.max
                (
                    1,

                    (
                        Math.max
                        (
                            1,

                            Math.max
                            (
                                0, 

                                smallestDistance - Constants.Vision.NOISY_DISTANCE_METERS
                            ) * Constants.Vision.DISTANCE_WEIGHT

                        ) * poseAmbiguityFactor

                    ) / (1 + ((estimation.targetsUsed.size() - 1) * Constants.Vision.TAG_PRESENCE_WEIGHT))
                );

        return Constants.Vision.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
    }

    /**
     * Takes a path from pathplanner, and turns it into a command to follow that path
     * 
     * @param path The pathplanner path to follow
     * @return The command for following that path
     * @author Unknown
     */
    public Command makePathFollowingCommand(PathPlannerPath path) 
    {
        return new FollowPathHolonomic
        (path, this::getEstimatedPose, this::getRobotRelativeSpeeds, this::driveRobotRelative,
                PATH_FOLLOWER_CONFIG,
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
                }, this
        );
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

        SmartDashboard.putNumber("distance to target", PhotonUtils.getDistanceToPose(getEstimatedPose(), new Pose2d(0.2, 5.6, new Rotation2d(0, 0))));
        

        // Do this in either robot or subsystem init
        SmartDashboard.putData("Field", m_field);

    }

}