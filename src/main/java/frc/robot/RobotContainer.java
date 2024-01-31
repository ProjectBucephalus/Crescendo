package frc.robot;

import java.sql.Driver;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.lib.util.COTSTalonFXSwerveConstants.SDS.MK3.driveRatios;
import frc.robot.Constants.AutoConstants;
import frc.robot.VisionCommands.aimToSpeaker;
import frc.robot.VisionCommands.multiTagPoseEstimatior;
import frc.robot.commands.*;
import frc.robot.commands.BuddyClimb.DeployBuddyClimber;
import frc.robot.commands.BuddyClimb.StopBuddyClimber;
import frc.robot.commands.Climber.MoveClimber;
import frc.robot.commands.Intake.IntakeDeploy;
import frc.robot.commands.Intake.IntakeSpit;
import frc.robot.commands.Intake.IntakeSuck;
import frc.robot.commands.Intake.MoveIntake;
import frc.robot.commands.Intake.MoveIntakeToPosition;
import frc.robot.commands.Intake.Flap.CloseFlap;
import frc.robot.commands.Intake.Flap.OpenFlap;
import frc.robot.commands.Intake.IntakeStow;
import frc.robot.commands.Shooter.ShooterRev;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakePosition;

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
    private final Joystick driver = new Joystick(0);
    private final Joystick coDriver = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
        // Swerve
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    
    private final POVButton      LOCK_CLIMBER_BUTTON   = new POVButton(driver, 0, 0); // The POV angles start at 0 in the up direction, and increase clockwise (e.g. right is 90, upper-left is 315).
    private final POVButton      UNLOCK_CLIMBER_BUTTON = new POVButton(driver, 180, 0);
    private final int            ALIGN_TO_SPEAKER      = XboxController.Axis.kLeftTrigger.value;
    private final JoystickButton ALIGN_TO_AMP          = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton INTAKE_BUTTON         = new JoystickButton(coDriver, XboxController.Button.kLeftBumper.value);
    private final int            BRAKE_AXIS            = XboxController.Axis.kRightTrigger.value;
    
    /* Co-Driver Buttons */
    private final int            SHOOT_BUTTON                  = XboxController.Axis.kLeftTrigger.value;
    private final JoystickButton FLAP_TOGGLE                   = new JoystickButton(coDriver, XboxController.Button.kLeftBumper.value);
    private final int            INTAKE_IN_BUTTON              = XboxController.Axis.kRightTrigger.value;
    private final JoystickButton INTAKE_OUT_BUTTON             = new JoystickButton(coDriver, XboxController.Button.kRightBumper.value);
    private final int            MANUAL_CLIMB_AXIS             = XboxController.Axis.kRightY.value;
    private final int            MANUAL_SHOTER_AXIS            = XboxController.Axis.kLeftY.value;
    private final POVButton      DEPLOY_BUDDY_CLIMBER          = new POVButton(coDriver, 270, 0);
    private final POVButton      AUTO_CLIMB_OUT                = new POVButton(coDriver, 0, 0);
    private final POVButton      STOP_BUDDY_CLIMBER         = new POVButton(coDriver, 90, 0);
    private final POVButton      AUTO_CLIMB_IN                 = new POVButton(coDriver, 180, 0);
    private final JoystickButton MANUAL_STOW_INTAKE            = new JoystickButton(coDriver, XboxController.Button.kA.value);
    private final JoystickButton MANUAL_SHOOTER_TO_AMP_POS     = new JoystickButton(coDriver, XboxController.Button.kB.value);
    private final JoystickButton MANUAL_INTAKE_TO_INTAKE_POS   = new JoystickButton(coDriver, XboxController.Button.kX.value);
    private final JoystickButton MANUAL_SHOOTER_TO_SPEAKER_POS = new JoystickButton(coDriver, XboxController.Button.kY.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Intake s_Intake = new Intake();
    private final Climber s_Climber = new Climber();

    PhotonCamera leftCamera = new PhotonCamera(Constants.Vision.leftCamName);
    PhotonCamera rightCamera = new PhotonCamera(Constants.Vision.rightCamName);
    PhotonCamera frontCamera = new PhotonCamera(Constants.Vision.frontCamName);
    private final Vision s_Vision = new Vision(leftCamera, rightCamera, frontCamera, s_Swerve);
    

    /* Autonomous */
    private final SendableChooser<Command> autoChooser;
    Field2d m_Field = new Field2d();
    ChoreoTrajectory traj;
    PathPlannerPath exampleChoreoTraj = PathPlannerPath.fromChoreoTrajectory("Trajectory");

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
                        () -> robotCentric.getAsBoolean(),
                        () -> -driver.getRawAxis(BRAKE_AXIS)));
        s_Vision.setDefaultCommand(new multiTagPoseEstimatior(s_Vision));
        //s_Intake.setDefaultCommand(new MoveIntake(s_Intake, () -> -coDriver.getRawAxis(MANUAL_SHOTER_AXIS)));   
        s_Climber.setDefaultCommand(new MoveClimber(s_Climber, () -> coDriver.getRawAxis(MANUAL_CLIMB_AXIS)));     

        configureButtonBindings();

        // NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
        // NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        // NamedCommands.registerCommand("print hello", Commands.print("hello"));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        traj = Choreo.getTrajectory("Trajectory");
        

        m_Field.getObject("traj").setPoses(traj.getInitialPose(), traj.getFinalPose());
        m_Field.getObject("trajPoses").setPoses(traj.getInitialPose(), traj.getFinalPose());

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
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        //INTAKE_BUTTON.toggleOnTrue(new IntakeSuck(s_Intake));
        INTAKE_OUT_BUTTON.whileTrue(new IntakeSuck(s_Intake));
        MANUAL_STOW_INTAKE.toggleOnTrue(new ShooterRev(s_Intake));
        
        /* Co-Driver Buttons */
        INTAKE_BUTTON.onTrue(new IntakeDeploy(s_Intake)).onFalse(new IntakeStow(s_Intake));
        MANUAL_INTAKE_TO_INTAKE_POS.onTrue(new MoveIntakeToPosition(s_Intake, IntakePosition.DEPLOYED));
        MANUAL_SHOOTER_TO_AMP_POS.onTrue(new MoveIntakeToPosition(s_Intake, IntakePosition.AMP));
        MANUAL_SHOOTER_TO_SPEAKER_POS.onTrue(new MoveIntakeToPosition(s_Intake, IntakePosition.SPEAKER));
        MANUAL_STOW_INTAKE.onTrue(new MoveIntakeToPosition(s_Intake, IntakePosition.STOWED));

        ALIGN_TO_AMP.whileTrue(new aimToSpeaker(s_Swerve));

        DEPLOY_BUDDY_CLIMBER.whileTrue(new DeployBuddyClimber(s_Intake)).whileFalse(new StopBuddyClimber(s_Intake));
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
