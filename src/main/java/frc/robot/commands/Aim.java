package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class Aim extends Command {
    private final Swerve s_Swerve;
    private final PhotonCamera cam1;

    private static final double ROTATION_SPEED_FACTOR = 10.0;
    private static final double MIN_ROTATION_SPEED = 0.1; // Adjust as needed
    private static final double ANGLE_TOLERANCE = 2.0; // Adjust as needed

    private double targetYaw;

    public Aim(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        this.cam1 = new PhotonCamera("cam1");
        addRequirements(s_Swerve);
    }

    @Override
public void execute() {
    var result = cam1.getLatestResult();
    double rotationSpeed;

    if (result.hasTargets()) {
        System.out.println("Seeing Targets");
        double yaw = result.getBestTarget().getYaw();
        Transform3d distance = result.getBestTarget().getBestCameraToTarget();
        System.out.println("Target Yaw: " + yaw);

        double radiansToGoal = Units.degreesToRadians(yaw);
        System.out.println("Radians To Goal: " + radiansToGoal);
        System.out.print("Distance:" + distance);

        // Adjust rotation speed based on AprilTag detection
        double targetYaw = 0.0; // Set your desired target angle here
        double yawError = targetYaw - yaw;

        // Proportional control to adjust rotation speed
        double proportionalTerm = 0.02; // Adjust this value based on experimentation
        rotationSpeed = ROTATION_SPEED_FACTOR * (-radiansToGoal + proportionalTerm * yawError);
    } else {
        System.out.println("No Targets");

        // Default rotation speed when no AprilTag is detected
        rotationSpeed = 0.2; // Set your desired default rotation speed here
    }

    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, rotationSpeed);
    s_Swerve.driveRobotRelative(speeds);

    System.out.println("------------------------------------------");
}


    private double calculateRotationSpeed(double radiansToGoal) {
    double remainingAngle = Math.abs(radiansToGoal);
    
    // Proportional control to gradually reduce rotation speed
    double proportionalSpeed = ROTATION_SPEED_FACTOR * remainingAngle;

    // Ensure rotation speed is not below the minimum speed
    return -Math.max(proportionalSpeed, MIN_ROTATION_SPEED) * Math.signum(radiansToGoal);
}


    private boolean isWithinTolerance(double radiansToGoal) {
        return Math.abs(radiansToGoal) < Units.degreesToRadians(ANGLE_TOLERANCE);
    }

    @Override
    public boolean isFinished() {
        var result = cam1.getLatestResult();
        return result.hasTargets() && isWithinTolerance(Units.degreesToRadians(result.getBestTarget().getYaw()));
    }
}
