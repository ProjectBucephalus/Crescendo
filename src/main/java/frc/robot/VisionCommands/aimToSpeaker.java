package frc.robot.VisionCommands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class aimToSpeaker extends Command {
    
    public Swerve s_Swerve;
    private AprilTagFieldLayout fieldLayout;

    public aimToSpeaker(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        SmartDashboard.putNumber("robot pose heading", calculateRequiredHeading().getDegrees());
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, calculateRequiredHeading().getRadians()*20);
        s_Swerve.driveRobotRelative(speeds);
        SmartDashboard.putNumber("robot pose heading", calculateRequiredHeading().getDegrees());
    }

    public Rotation2d calculateRequiredHeading() {
        var pose = s_Swerve.getEstimatedPose();
        return PhotonUtils.getYawToPose(pose, new Pose2d(0, 5.6, new Rotation2d(0,0)));
        
    }
}
