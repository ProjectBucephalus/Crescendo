package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

/**
 * aim command
 * @author 5985
 */
public class Aim extends Command {
    private final Swerve s_Swerve;
    private final Intake s_Intake;
    private final PhotonCamera cam2;

    private static final double ROTATION_SPEED_FACTOR = 10.0;
    private static final double ANGLE_TOLERANCE = 2.0; // Adjust as needed

    public boolean teamRed = false;
    public int[] redTargets = {9, 10, 3, 4, 5, 11, 12, 13}; // [source, source, speaker-side, speaker-centre, amp, stage, stage, stage]
    public int[] blueTargets = {1, 2, 8, 7, 6, 14, 15, 16}; // [source, source, speaker-side, speaker-centre, amp, stage, stage, stage]
    public int[] teamTargets = new int[8];

    private double absolutePitch;

    private double horizontalDistance;

    private double targetPitch;

    private double speakerTagHeightOverCamera = Constants.speakerTagHeight - Constants.cameraHeightOverGround;

    private double targetHeightOverCamera = speakerTagHeightOverCamera + Constants.targetHeightOverTag;

    private double shooterAngle;

    public Aim(Swerve s_Swerve, Intake s_Intake) 
    {
        this.s_Swerve = s_Swerve;
        this.s_Intake = s_Intake;
        this.cam2 = new PhotonCamera("cam2");
        addRequirements(s_Swerve, s_Intake);
    }

    // Add initalise function here
    // Need to set team boolean value, and coppy team target list.
    public void initalize() 
    {
        if (teamRed) 
        {
            teamTargets = redTargets;
        } 
        else 
        {
            teamTargets = blueTargets;
        }
    }


    @Override
    public void execute() 
    {
        var result = cam2.getLatestResult();
        double rotationSpeed;

        if (result.hasTargets()) 
        {
            System.out.println("Seeing Targets");
            // Centre tags of speaker targets are ID 4 (Red) & 7 (Blue), stored in index 3 of team target lists.
            var target = getTargetFromID(result, teamTargets[3]);
            double yaw = target.getYaw();
            double pitch = target.getPitch();

            //horizontalDistance = tagHeight / tan absolutePitch
            absolutePitch = Constants.cameraPitchOffset + pitch;
            horizontalDistance = speakerTagHeightOverCamera / Math.tan(Units.degreesToRadians(absolutePitch));
            targetPitch = Math.atan(targetHeightOverCamera / Math.max(1,horizontalDistance));
            targetPitch = Units.radiansToDegrees(targetPitch);
            shooterAngle = targetPitch + Constants.horizontalShooterAngle;
            Transform3d distance = target.getBestCameraToTarget();
            
            System.out.println("Target Yaw: " + yaw);
            System.out.println("Target Pitch: " + pitch);
            System.out.println("Horizontal Distance To Target: " + Math.floor(horizontalDistance));
            System.out.println("Absolute Target Pitch: " + Math.floor(targetPitch));

            double radiansToGoal = Units.degreesToRadians(yaw);
            System.out.println("Radians To Goal: " + radiansToGoal);
            System.out.print("Distance:" + distance);

            // Adjust rotation speed based on AprilTag detection
            double targetYaw = 0.0; // Set your desired target angle here
            double yawError = targetYaw - yaw;

            // Proportional control to adjust rotation speed
            double proportionalTerm = 0.02; // Adjust this value based on experimentation
            rotationSpeed = ROTATION_SPEED_FACTOR * (-radiansToGoal + proportionalTerm * yawError);

        } 
        else 
        {
            System.out.println("No Targets");

            // Default rotation speed when no AprilTag is detected
            rotationSpeed = 0.2; // Set your desired default rotation speed here
        }

        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, rotationSpeed);
        s_Swerve.driveRobotRelative(speeds);
        s_Intake.moveArmToAngle(shooterAngle);

        System.out.println("------------------------------------------");
    }

    private boolean isWithinTolerance(double radiansToGoal) 
    {
        return Math.abs(radiansToGoal) < Units.degreesToRadians(ANGLE_TOLERANCE);
    }

    @Override
    public boolean isFinished() 
    {
        var result = cam2.getLatestResult();
        return result.hasTargets() && isWithinTolerance(Units.degreesToRadians(result.getBestTarget().getYaw()));
    }

    private PhotonTrackedTarget getTargetFromID(PhotonPipelineResult result, int goal_ID) 
    {
        var targets = result.getTargets();
            for(int i = 0; i< targets.size(); i++)
            {
                var target = targets.get(i);
                if(target.getFiducialId() == goal_ID)
                
                {
                    return target;
                }
            }
            return targets.get(0);
    }
}
