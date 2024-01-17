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
            System.out.println("Seeing Targets");
            System.out.println("Target Yaw:");
            System.out.println(result.getBestTarget().getYaw());
            double radiansToGoal = Units.degreesToRadians(result.getBestTarget().getYaw());
            System.out.println("Radians To Goal:");
            System.out.println(radiansToGoal);
            double rotationSpeed = 20 * -radiansToGoal;
            ChassisSpeeds speeds = new ChassisSpeeds(0, 0, rotationSpeed); 
            s_Swerve.driveRobotRelative(speeds);
            System.out.println("------------------------------------------");
        } else {
            System.out.println("No Targets");
        }
    }
}