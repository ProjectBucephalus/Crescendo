package frc.robot.VisionCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class aimToSpeaker extends Command {
    
    public Swerve s_Swerve;

    public aimToSpeaker(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        SmartDashboard.putNumber("robot pose heading", calculateRequiredHeading());
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, edu.wpi.first.math.util.Units.degreesToRadians(calculateRequiredHeading()/10));
        s_Swerve.driveRobotRelative(speeds);
        SmartDashboard.putNumber("robot pose heading", calculateRequiredHeading());
    }

    public double calculateRequiredHeading() {
        var X = s_Swerve.getEstimatedPose().getX();
        var Y = s_Swerve.getEstimatedPose().getX();
        var theta = Math.atan(Y/(5.54-X));
        return theta;
    }
}
