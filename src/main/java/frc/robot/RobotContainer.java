package frc.robot;

import java.util.ArrayList;

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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton aim = new JoystickButton(driver, XboxController.Button.kA.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

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
                        () -> robotCentric.getAsBoolean()));

        // Configure the button bindings
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
        aim.whileTrue(new Aim(s_Swerve));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

     public Command getAutonomousCommand() {
        var thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        s_Swerve.resetOdometry(traj.getInitialPose());
    
        // Command swerveCommand = Choreo.choreoSwerveCommand(
        //     traj, // Choreo trajectory from above
        //     s_Swerve::getPose, // A function that returns the current field-relative pose of the robot: your
        //                            // wheel or vision odometry
        //     new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // PIDController for field-relative X
        //                                                                                // translation (input: X error in meters,
        //                                                                                // output: m/s).
        //     new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0), // PIDController for field-relative Y
        //                                                                                // translation (input: Y error in meters,
        //                                                                                // output: m/s).
        //     thetaController, // PID constants to correct for rotation
        //                      // error
        //     (ChassisSpeeds speeds) -> s_Swerve.ChoreoDrive( // needs to be robot-relative
        //         speeds.vxMetersPerSecond,
        //         speeds.vyMetersPerSecond,
        //         speeds.omegaRadiansPerSecond,
        //         false),
        //     true, // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
        //     s_Swerve // The subsystem(s) to require, typically your drive subsystem only
        // );
    
        // return Commands.sequence(
        //     Commands.runOnce(() -> s_Swerve.resetOdometry(traj.getInitialPose())),
        //     swerveCommand,
        //     s_Swerve.run(() -> s_Swerve.ChoreoDrive(0, 0, 0, false))
        // );

        return null;
      }

    public void periodic() {
        m_Field.setRobotPose(s_Swerve.getPose());
    }
}
