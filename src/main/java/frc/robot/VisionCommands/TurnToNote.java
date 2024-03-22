package frc.robot.VisionCommands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SwerveConstants;
import frc.robot.subsystems.NoteVision;
import frc.robot.subsystems.Swerve;

public class TurnToNote extends Command {
    private final Swerve s_Swerve;
    private NoteVision s_NoteVision;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier brakeSup;

    public TurnToNote(Swerve driveSubsystem, NoteVision s_NoteVision, DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier brakeSup) {
        s_Swerve = driveSubsystem;
        this.s_NoteVision = s_NoteVision;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.brakeSup = brakeSup;

        SmartDashboard.putNumber("Radians Times", 10);
        SmartDashboard.putNumber("To power", 1);

    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double turningVal = 0;
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double brakeVal = MathUtil.applyDeadband(brakeSup.getAsDouble(),
                Constants.stickDeadband);
        Translation2d translation = new Translation2d(translationVal,
                strafeVal).times(SwerveConstants.maxSpeed);
        try {
            SmartDashboard.putNumber("Note Position, Yaw", s_NoteVision.getNotesYaw().get(0));
            SmartDashboard.putNumber("Note Position, X",
                    s_NoteVision.getNotes(s_Swerve.getEstimatedPose()).get(0).getX());
            SmartDashboard.putNumber("Note Position, Y",
                    s_NoteVision.getNotes(s_Swerve.getEstimatedPose()).get(0).getY());
            SmartDashboard
                    .putNumber("Note Position, requiredHeading",
                            calculateRequiredHeading(new Pose2d(
                                    s_NoteVision.getNotes(s_Swerve.getEstimatedPose()).get(0).getX(),
                                    s_NoteVision.getNotes(s_Swerve.getEstimatedPose()).get(0).getY(), new Rotation2d()))
                                    .getDegrees());

            /* Remove the roate by 180 when the camera gets moved */
            turningVal = -Math.copySign(Math.pow(calculateRequiredHeading(new Pose2d(
                    s_NoteVision.getNotes(s_Swerve.getEstimatedPose()).get(0).getX(),
                    s_NoteVision.getNotes(s_Swerve.getEstimatedPose()).get(0).getY(),
                    new Rotation2d())).rotateBy(Rotation2d.fromDegrees(180)).getRadians()
                    * SmartDashboard.getNumber("Radians Times", 0), SmartDashboard.getNumber("To power", 0)),
                    calculateRequiredHeading(new Pose2d(
                            s_NoteVision.getNotes(s_Swerve.getEstimatedPose()).get(0).getX(),
                            s_NoteVision.getNotes(s_Swerve.getEstimatedPose()).get(0).getY(),
                            new Rotation2d())).rotateBy(Rotation2d.fromDegrees(180)).getRadians() * 70);

            // tuningVal;
        } catch (Exception e) {
            /* Sometimes we may loose the note don't really want the code to crash :) */
        }

        // s_Swerve.driveRobotRelative(translation, -turningVal, true, brakeVal);

        /* We set this to true so that we only use this vision drive method to drive. */
        s_Swerve.setVisionAlignmentBool(true);
        s_Swerve.visionDrive(translation, turningVal, true, brakeVal);

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
