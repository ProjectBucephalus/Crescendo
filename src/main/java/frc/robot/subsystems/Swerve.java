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
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDriveOdometry swerveOdometryEstimated;

    public SwerveDrivePoseEstimator poseEstimator;
    public PhotonPoseEstimator photonPoseEstimatorFront;
    public PhotonPoseEstimator photonPoseEstimatorBack;
    public PhotonCamera frontCam = new PhotonCamera(Constants.Vision.backCamName);
    public PhotonCamera backCam = new PhotonCamera(Constants.Vision.frontCamName);

    public boolean usingVisionAlignment = false;

    final AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); // see docs for how to
                                                                                                // do this better and
                                                                                                // set origin for red
                                                                                                // alliance

    private final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
            new PIDConstants(Constants.AutoConstants.kPXController),
            new PIDConstants(Constants.AutoConstants.kPYController), Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.Swerve.trackWidth, new ReplanningConfig());

    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public Pose2d pose;

    public Swerve() {
        gyro = new Pigeon2(Constants.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        swerveOdometryEstimated = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(),
                getModulePositions());

        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(),
                Constants.Vision.STATE_STANDARD_DEVIATIONS,
                Constants.Vision.VISION_MEASUREMENT_STANDARD_DEVIATIONS);

        photonPoseEstimatorFront = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCam,
                Constants.Vision.frontCamToRobot);
        photonPoseEstimatorBack = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCam,
                Constants.Vision.backCamToRobot);

        photonPoseEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonPoseEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
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
     * TODO i dont know how the swerve works, todo docs
     * 
     * @param translation
     * @param rotation
     * @param fieldRelative
     * @param isOpenLoop
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop,
            double brakeVal) {
        if (!usingVisionAlignment) {
            SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                    fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                            translation.getX(),
                            translation.getY(),
                            rotation,
                            getHeading())
                            : new ChassisSpeeds(
                                    translation.getX(),
                                    translation.getY(),
                                    rotation));
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                    Constants.Swerve.maxSpeed * (map(brakeVal, 0, 1, Constants.Swerve.brakeIntensity, 1)));
            for (SwerveModule mod : mSwerveMods) {
                mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
            }
        }
    }

    public void visionDrive(Translation2d translation, double rotation, boolean isOpenLoop, double brakeVal) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading()));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                Constants.Swerve.maxSpeed * (map(brakeVal, 0, 1, Constants.Swerve.brakeIntensity, 1)));
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /**
     * TODO i dont know how the swerve works, todo docs
     * 
     * @param xSpeed
     * @param ySpeed
     * @param rot
     * @param fieldRelative
     */
    public void ChoreoDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, gyro.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }

    /**
     * Used by SwerveControllerCommand in Auto
     * 
     * @param desiredStates
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    /**
     * TODO i dont know how the swerve works, todo docs
     * 
     * @param pose
     */
    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public Rotation2d getGyro() {
        return gyro.getRotation2d();
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    /**
     * TODO docs
     */
    public static double map(double valueCoord1,
            double startCoord1, double endCoord1,
            double startCoord2, double endCoord2) {

        double R = (endCoord2 - startCoord2) / (endCoord1 - startCoord1);
        double y = startCoord2 + (valueCoord1 * R) + R;
        return (y);
    }

    /**
     * TODO i dont know how the swerve works, todo docs
     * 
     * @param pose
     */
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void resetEstimatedOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Set to true to ignore roatational values from the controller and use values
     * from vision
     * 
     * @param newVal Set to true to ignore rotational inputs and use roation from
     *               driveRobotRelative(). Set to false to revert.
     */
    public void setVisionAlignmentBool(boolean newVal) {
        usingVisionAlignment = newVal;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * TODO i dont know how the swerve works, todo docs
     * 
     * @param robotRelativeSpeeds
     */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
        double smallestDistance = Double.POSITIVE_INFINITY;
        for (var target : estimation.targetsUsed) {
            var t3d = target.getBestCameraToTarget();
            var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
            if (distance < smallestDistance)
                smallestDistance = distance;
        }
        double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
                ? 1
                : Math.max(
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

    public Command makePathFollowingCommand(PathPlannerPath path) {

        return new FollowPathHolonomic(path, this::getEstimatedPose, this::getRobotRelativeSpeeds, this::driveRobotRelative,
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
                }, this);
    }

    @Override
    /**
     * TODO i dont know how the swerve works, todo docs
     */
    public void periodic() {

        swerveOdometry.update(getGyroYaw(), getModulePositions());
        swerveOdometryEstimated.update(getGyroYaw(), getModulePositions());

        final Optional<EstimatedRobotPose> optionalEstimatedPoseFront = photonPoseEstimatorFront.update();
        if (optionalEstimatedPoseFront.isPresent()) {
            SmartDashboard.putBoolean("Using Vision", true);
            final EstimatedRobotPose estimatedPose = optionalEstimatedPoseFront.get();
            poseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds,
                    confidenceCalculator(estimatedPose));
        } else {
            SmartDashboard.putBoolean("Using Vision", false);
        }

        final Optional<EstimatedRobotPose> optionalEstimatedPoseBack = photonPoseEstimatorBack.update();
        if (optionalEstimatedPoseBack.isPresent()) {
            SmartDashboard.putBoolean("Using Vision", true);
            final EstimatedRobotPose estimatedPose = optionalEstimatedPoseBack.get();
            poseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds,
                    confidenceCalculator(estimatedPose));
        } else {
            SmartDashboard.putBoolean("Using Vision", false);
        }

        poseEstimator.update(getGyro(), getModulePositions());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        SmartDashboard.putNumber("Pose X ", getPose().getX());
        SmartDashboard.putNumber("Pose Y ", getPose().getY());
        SmartDashboard.putNumber("Pose X (Estimated)", getEstimatedPose().getX());
        SmartDashboard.putNumber("Pose Y (Estimated)", getEstimatedPose().getY());
        SmartDashboard.putNumber("Rotaton (Estimated)", getEstimatedPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Pose X (PhotonPoseEstimator)", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Pose Y (PhotonPoseEstimator)", poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Rotaton (PhotonPoseEstimator)",
                poseEstimator.getEstimatedPosition().getRotation().getDegrees());

    }

}