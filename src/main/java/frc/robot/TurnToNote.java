package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Utilities.Limelight;
import frc.robot.subsystems.Swerve;

public class TurnToNote extends Command {
    private final Swerve s_Swerve;
    private Limelight m_lime;
    private double tx;
    private final double tuningVal = .29;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier brakeSup;

    public TurnToNote(Swerve driveSubsystem, Limelight limeu, DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier brakeSup) {
        s_Swerve = driveSubsystem;
        m_lime = limeu;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.brakeSup = brakeSup;

    }

    @Override
    public void initialize() {
        m_lime.enableVision();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        // double brakeVal = MathUtil.applyDeadband(brakeSup.getAsDouble(), Constants.stickDeadband);
        // Translation2d translation = new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed);

        tx = m_lime.getAngleToTarget();
        var turningVal = Math.copySign(Math.pow(Math.abs(tx), 0.0525), tx) * tuningVal;
        //s_Swerve.driveRobotRelative(translation, -turningVal, true, brakeVal);
        s_Swerve.setVisionAlignmentBool(true);
        s_Swerve.driveRobotRelative(new ChassisSpeeds(translationVal, strafeVal, -turningVal));

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
        //     return true;
        // }
        return false;
    }
}
