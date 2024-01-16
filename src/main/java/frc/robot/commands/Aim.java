package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class Aim extends Command {
    private Swerve s_Swerve; 
    
    PhotonCamera cam1 = new PhotonCamera("cam1");    
    
     public Aim(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }
    
    public void execute() {
        var result = cam1.getLatestResult();
        if (result.hasTargets()) {
        double radiansToGoal = Units.degreesToRadians(result.getBestTarget().getYaw());

        var speeds = new ChassisSpeeds(0, 0, radiansToGoal); 
        s_Swerve.driveRobotRelative(speeds);
        } else {
            System.out.println("No Targets");
        }
    }
}
