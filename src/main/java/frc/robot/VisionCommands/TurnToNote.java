package frc.robot.VisionCommands;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SwerveConstants;
import frc.robot.subsystems.NoteVision;
import frc.robot.subsystems.Swerve;

public class TurnToNote extends Command 
{
    private final Swerve s_Swerve;
    private NoteVision s_NoteVision;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private DoubleSupplier brakeSup;
    private XboxController xbox; 

    public TurnToNote(XboxController xbox, Swerve driveSubsystem, NoteVision s_NoteVision, DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            DoubleSupplier brakeSup) {
        s_Swerve = driveSubsystem;
        this.s_NoteVision = s_NoteVision;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.brakeSup = brakeSup;
        this.xbox = xbox;
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        double brakeVal = MathUtil.applyDeadband(brakeSup.getAsDouble(),
                Constants.stickDeadband);
        
        double turningVal = rotationVal * SwerveConstants.maxAngularVelocity;

        Translation2d translation = new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed);
        List<Translation2d> notes = s_NoteVision.getNotes(s_Swerve.getEstimatedPose());
        double noteHeading = 0;
        try 
        {
            if (notes.size() > 0) 
            {   
                SmartDashboard.putBoolean("Seeing note?", true);
                noteHeading = calculateRequiredHeading(new Pose2d(notes.get(0).getX(), notes.get(0).getY(), new Rotation2d())).getRadians();
                SmartDashboard.putNumber("Note Position, requiredHeading", noteHeading);

                turningVal = -Math.copySign
                (
                    Math.pow(noteHeading * Constants.Vision.noteTurnScalarGain, Constants.Vision.noteTurnPowerGain),
                    noteHeading
                );
                xbox.setRumble(RumbleType.kBothRumble, 0.5
                );
            }
            else 
            {
                SmartDashboard.putBoolean("Seeing note?", false);
                xbox.setRumble(RumbleType.kBothRumble, 0);
            }
        } 
        catch (Exception e) 
            {}

        // s_Swerve.driveRobotRelative(translation, -turningVal, true, brakeVal);

        /* We set this to true so that we only use this vision drive method to drive. */
        SmartDashboard.putNumber("RotationVal", rotationVal);
        SmartDashboard.putNumber("TurningVal", turningVal);
        s_Swerve.setVisionAlignmentBool(true);
        s_Swerve.visionDrive(translation, turningVal, false, true, brakeVal);
    }

    public Rotation2d calculateRequiredHeading(Pose2d p) {
        var pose = s_Swerve.getEstimatedPose();
        // TODO Change with alliances
        return PhotonUtils.getYawToPose(pose, p);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // m_lime.disableVision();
        s_Swerve.setVisionAlignmentBool(false);
        xbox.setRumble(RumbleType.kBothRumble, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // if (Math.abs(m_joy.getX()) > 0.3 || Math.abs(m_joy.getY()) > 0.3) {
        // return true;
        // }
        return false;
    }
}
