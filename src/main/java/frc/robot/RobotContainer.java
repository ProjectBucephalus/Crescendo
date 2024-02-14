package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.VisionCommands.AimToSpeakerNoDrive;
import frc.robot.VisionCommands.aimToSpeaker;
import frc.robot.VisionCommands.multiTagPoseEstimatior;
import frc.robot.commands.*;
import frc.robot.commands.BuddyClimb.DeployBuddyClimber;
import frc.robot.commands.BuddyClimb.StopBuddyClimber;
import frc.robot.commands.Climber.ClimberExtend;
import frc.robot.commands.Climber.ClimberRetract;
import frc.robot.commands.Climber.LockClimber;
import frc.robot.commands.Climber.MoveClimber;
import frc.robot.commands.Climber.UnlockClimber;
import frc.robot.commands.Intake.IntakeAndDeployPivot;
import frc.robot.commands.Intake.IntakeSpit;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Intake.IntakeSuck;
import frc.robot.commands.Intake.MovePivotToPosition;
import frc.robot.commands.Intake.StopIntakeAndStow;
import frc.robot.commands.Shooter.ShooterIdle;
import frc.robot.commands.Shooter.ShooterRev;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Pivot.PivotPosition;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController coDriver = new CommandXboxController(1); // declare xbox on ds port 1

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    // Swerve
    // private final JoystickButton zeroGyro = new JoystickButton(driver,
    // XboxController.Button.kY.value);
    // private final JoystickButton robotCentric = new JoystickButton(driver,
    // XboxController.Button.kLeftBumper.value);

    private final int BRAKE_AXIS = XboxController.Axis.kRightTrigger.value;

    private final int MANUAL_CLIMB_AXIS = XboxController.Axis.kLeftY.value;
    private final int MANUAL_SHOTER_AXIS = XboxController.Axis.kRightY.value;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Intake s_Intake = new Intake();
    private final Pivot s_Pivot = new Pivot();
    private final Climber s_Climber = new Climber();
    private final Shooter s_Shooter = new Shooter();

    private final NoteVision m_noteVision = new NoteVision();

    /* Autonomous */
    private final SendableChooser<Command> autoChooser;
    Field2d m_Field = new Field2d();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> driver.leftBumper().getAsBoolean(),
                        () -> -driver.getRawAxis(BRAKE_AXIS)));
        // s_Vision.setDefaultCommand(new multiTagPoseEstimatior(s_Vision));
        // s_Intake.setDefaultCommand(new MoveIntake(s_Intake, () ->
        // -coDriver.getRawAxis(MANUAL_SHOTER_AXIS)));
        s_Climber.setDefaultCommand(new MoveClimber(s_Climber, () -> -coDriver.getRawAxis(MANUAL_CLIMB_AXIS)));

        configureButtonBindings();

        NamedCommands.registerCommand("IntakeAndDeployPivot", new IntakeAndDeployPivot(s_Pivot, s_Intake));
        NamedCommands.registerCommand("StopIntakeAndStow", new StopIntakeAndStow(s_Pivot, s_Intake));
        NamedCommands.registerCommand("AimToSpeaker", new AimToSpeakerNoDrive(s_Swerve, s_Pivot));
        NamedCommands.registerCommand("Start Shooter", new ShooterRev(s_Shooter));
        NamedCommands.registerCommand("Stop Shooter", new ShooterIdle(s_Shooter));
        NamedCommands.registerCommand("Intake Suck", new IntakeSuck(s_Intake));
        NamedCommands.registerCommand("Intake Stop", new IntakeStop(s_Intake));
        NamedCommands.registerCommand("Move to Base Speaker Angle", new MovePivotToPosition(s_Pivot, PivotPosition.SPEAKER_MANUAL));
        NamedCommands.registerCommand("Stow Pivot", new MovePivotToPosition(s_Pivot, PivotPosition.STOWED));
        
        // NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        // NamedCommands.registerCommand("print hello", Commands.print("hello"));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        SmartDashboard.putData(m_Field);
        final var visionTab = Shuffleboard.getTab("Vision");

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
    private void configureButtonBindings() {
        /* Driver Buttons */
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        driver.leftTrigger().whileTrue(new aimToSpeaker(s_Swerve, () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(BRAKE_AXIS), s_Pivot));
        // driver.rightBumper() .whileTrue(new IntakeSuck(s_Intake));
        // driver.rightBumper() .onTrue(new IntakeSpit(s_Intake)).onFalse(new
        // IntakeStop(s_Intake));
        driver.rightBumper().onTrue(new IntakeAndDeployPivot(s_Pivot, s_Intake))
                .onFalse(new StopIntakeAndStow(s_Pivot, s_Intake));
        // driver.povUp().onTrue(new unlockClimber(s_Climber));
        // driver.povDown().onTrue(new lockClimber(s_Climber));

        /* Co-Driver Buttons */

        coDriver.leftTrigger().whileTrue(new ShooterRev(s_Shooter)).whileFalse(new ShooterIdle(s_Shooter));
        coDriver.rightTrigger().whileTrue(new IntakeSuck(s_Intake));

        coDriver.x().onTrue(new MovePivotToPosition(s_Pivot, PivotPosition.DEPLOYED));
        coDriver.b().onTrue(new MovePivotToPosition(s_Pivot, PivotPosition.AMP));
        coDriver.y().onTrue(new MovePivotToPosition(s_Pivot, PivotPosition.SPEAKER));
        coDriver.a().onTrue(new MovePivotToPosition(s_Pivot, PivotPosition.STOWED));
      
        coDriver.povRight().onTrue(new DeployBuddyClimber(s_Climber));
        coDriver.povLeft().onTrue(new StopBuddyClimber(s_Climber));
        coDriver.povDown()     .onTrue(new ClimberRetract(s_Climber));
        coDriver.povUp()       .onTrue(new ClimberExtend(s_Climber));
        coDriver.rightBumper() .whileTrue(new IntakeSpit(s_Intake));

        SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
            Pose2d currentPose = s_Swerve.getEstimatedPose();

            // The rotation component in these poses represents the direction of travel
            Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
            Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)),
                    new Rotation2d());

            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
            PathPlannerPath path = new PathPlannerPath(
                    bezierPoints,
                    new PathConstraints(
                            4.0, 4.0,
                            Units.degreesToRadians(360), Units.degreesToRadians(540)),
                    new GoalEndState(0.0, currentPose.getRotation()));

            // Prevent this path from being flipped on the red alliance, since the given
            // positions are already correct
            path.preventFlipping = true;

            AutoBuilder.followPath(path).schedule();
        }));

        

        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
