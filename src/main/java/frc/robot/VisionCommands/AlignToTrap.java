package frc.robot.VisionCommands;

import java.sql.Driver;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.SwerveConstants;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Pivot.PivotPosition;
import frc.robot.subsystems.Shooter.ShooterState;

public class AlignToTrap extends Command {

    public Swerve s_Swerve;

    private Pose2d shootingPose;
    private Transform2d targetLocation;

    public AlignToTrap(Swerve s_Swerve, Transform2d targetLocation) {
        this.s_Swerve = s_Swerve;
        this.targetLocation = targetLocation;
    }

    @Override
    public void initialize() {
        s_Swerve.setVisionAlignmentBool(true);
    }

    @Override
    public void execute() 
    {   
        shootingPose = new Pose2d(targetLocation.getTranslation(), targetLocation.getRotation());
        Transform2d distanceToShootingPos = s_Swerve.getEstimatedPose().minus(FieldConstants.flipPose(shootingPose));
        Translation2d translation = new Translation2d(distanceToShootingPos.getY(), distanceToShootingPos.getX()).times(SwerveConstants.maxSpeed);

        s_Swerve.visionDrive(translation, shootingPose.getRotation().getRadians(), true, true, 0);
    }

    public boolean isFinished() 
    {
        return true;
    }

    @Override
    public void end(boolean end) 
    {
        s_Swerve.setVisionAlignmentBool(false);

        /*
         * Make sure we do this so that other manual alignment functions work. It should
         * only be false if we are currently auto aligning and not withing alignemnt
         * tolerance.
         */
        s_Swerve.setWithinRequiredHeading(true);
    } 
    
}
