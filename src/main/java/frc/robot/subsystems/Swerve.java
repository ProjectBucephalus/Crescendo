package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.IDConstants;
import frc.robot.SwerveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

import javax.swing.plaf.basic.BasicSliderUI.TrackListener;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase 
{
    // For showing the stored tip postition on AdvantageScope
    private final Field2d storedPoseDisplay = new Field2d();

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

    /** List of swerve module motors */
    public SwerveModule[] mSwerveMods;
    
    /** Robot's gyro [X Right, Y Forward, Z Up] */
    public Pigeon2 gyro;
    
    /** Robot's field relative position */
    public Pose2d pose;

    private Optional<EstimatedRobotPose> visionEstimatedPoseFront, visionEstimatedPoseBack;
    private EstimatedRobotPose estimatedRobotPose;

    /** Tracks whether the robot was tipped last cycle */
    private boolean trackTipped = false;

    /** Stores the pose when the robot tips */
    private Pose2d storePose = new Pose2d();

    

    public Swerve(SendableChooser<Pose2d> m_startLocation) 
    {
        // Define and initialise gyro, as well as applying config
        gyro = new Pigeon2(IDConstants.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        SmartDashboard.putData("TipField", storedPoseDisplay);

        // Define and initialise list of swerve modules
        mSwerveMods = new SwerveModule[] 
        {
                new SwerveModule(0, SwerveConstants.Mod0.constants),
                new SwerveModule(1, SwerveConstants.Mod1.constants),
                new SwerveModule(2, SwerveConstants.Mod2.constants),
                new SwerveModule(3, SwerveConstants.Mod3.constants)
        };

        // Define and initialise pose estimator
        swerveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions());
        poseEstimator = new SwerveDrivePoseEstimator
        (
            SwerveConstants.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d(),
            Constants.Vision.stateStandardDeviations,
            Constants.Vision.visionMeasurementStandardDeviations
        );

        // Define and initialise PhotonPoseEstimators
        photonPoseEstimatorFront = new PhotonPoseEstimator
        (layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCam, Constants.Vision.frontCamToRobot);
        
        photonPoseEstimatorBack = new PhotonPoseEstimator
        (layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCam, Constants.Vision.backCamToRobot);
        
        photonPoseEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonPoseEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Configures the AutoBuilder
        AutoBuilder.configureHolonomic
        (
                this::getEstimatedPose, // Robot pose supplier
                this::resetEstimatedOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig // HolonomicPathFollowerConfig, this should likely live in your Constants class
                ( 
                        new PIDConstants(10, 0.3, 0.3), // Translation PID constants
                        new PIDConstants(10, 0.3, 0.3), // Rotation PID constants
                        100, // Max module speed, in m/s
                        0.34, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> 
                {
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
        SmartDashboard.putNumber("BrakeVal", brakeVal);
        SmartDashboard.putBoolean("Egotistic?", !fieldRelative);
        if (!usingVisionAlignment) 
        {
            SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates
            (
                fieldRelative ? 
                ChassisSpeeds.fromFieldRelativeSpeeds
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

    /**
     * Swerve magic
     * 
     * @param translation
     * @param rotation
     * @param isOpenLoop
     * @param brakeVal
     * @author 5985
     */
    public void visionDrive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, double brakeVal) 
    {
        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates
        (
            fieldRelative ? 
            ChassisSpeeds.fromFieldRelativeSpeeds
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
        swerveOdometry.resetPosition
        (getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
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
    public static double map(double valueCoord1, double startCoord1, double endCoord1, double startCoord2, double endCoord2) 
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
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) 
    {
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
            (estimation.targetsUsed.get(0).getPoseAmbiguity() + Constants.Vision.poseAmbiguityShifter)
            * Constants.Vision.poseAmbiguityMultilplier
        );
        
        double confidenceMultiplier = Math.max
        (
            1,
            (
                Math.max
                (
                    1,
                    Math.max
                    (0, smallestDistance - Constants.Vision.noisyDistanceMeters) * Constants.Vision.distanceWeight
                )
                * poseAmbiguityFactor
            )
            / 
            (1 + ((estimation.targetsUsed.size() - 1) * Constants.Vision.tagPresenceWeight))
        );

        return Constants.Vision.visionMeasurementStandardDeviations.times(confidenceMultiplier);
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
        return AutoBuilder.followPath(path);
    }
 
    @Override
    /**
     * TODO docs
     */
    public void periodic() 
    {   
        

        if((gyro.getRoll().getValueAsDouble() < 10 && gyro.getRoll().getValueAsDouble() > -10) && (gyro.getPitch().getValueAsDouble() < 10 && gyro.getPitch().getValueAsDouble() > -10))
        {
            
            if (trackTipped == true) 
            {   
               swerveOdometry.resetPosition(getGyro(), getModulePositions(), storePose);
               poseEstimator.resetPosition(getGyro(), getModulePositions(), storePose);
            }
            trackTipped = false;
            swerveOdometry.update(getGyroYaw(), getModulePositions());
            poseEstimator.update(getGyro(), getModulePositions());
        }
        else if (trackTipped == false)
        {   
            trackTipped = true;
            storePose = getEstimatedPose();
        }

        m_field.setRobotPose(getEstimatedPose());

        // Do this in either robot periodic or subsystem periodic
        storedPoseDisplay.setRobotPose(storePose);

        SmartDashboard.putNumber("Roll", gyro.getRoll().getValueAsDouble());
        SmartDashboard.putNumber("Pitch", gyro.getPitch().getValueAsDouble());

        //final Optional<EstimatedRobotPose> 
        visionEstimatedPoseFront = photonPoseEstimatorFront.update();
        if (visionEstimatedPoseFront.isPresent()) 
        {
            SmartDashboard.putBoolean("Using Front Vision", true);
            estimatedRobotPose = visionEstimatedPoseFront.get();
            poseEstimator.addVisionMeasurement
            (estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds, confidenceCalculator(estimatedRobotPose));
        } 
        else 
        {
            SmartDashboard.putBoolean("Using Front Vision", false);
        }

        visionEstimatedPoseBack = photonPoseEstimatorBack.update();
        if (visionEstimatedPoseBack.isPresent()) 
        {
            SmartDashboard.putBoolean("Using Back Vision", true);
            estimatedRobotPose = visionEstimatedPoseBack.get();
            poseEstimator.addVisionMeasurement
            (estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds, confidenceCalculator(estimatedRobotPose));
        } 
        else 
        {
            SmartDashboard.putBoolean("Using Back Vision", false);
        }

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        SmartDashboard.putNumber("Pose X (Estimated)", getEstimatedPose().getX());
        SmartDashboard.putNumber("Pose Y (Estimated)", getEstimatedPose().getY());
        SmartDashboard.putNumber("Rotaton (Estimated)", getEstimatedPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Rotaton (flipped)", getEstimatedPose().getRotation().getDegrees() + 180);

        SmartDashboard.putBoolean("usingVisionAlignment", usingVisionAlignment);

        SmartDashboard.putNumber("distance to target", PhotonUtils.getDistanceToPose(getEstimatedPose(), FieldConstants.flipPose(FieldConstants.translationToPose2d(FieldConstants.SPEAKER))));
        

        // Do this in either robot or subsystem init
        SmartDashboard.putData("Field", m_field);

    }
}