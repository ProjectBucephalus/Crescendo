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
import frc.robot.commands.Climber.MoveClimber;
import frc.robot.commands.Intake.IntakeDeploy;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.IntakeUp;
import frc.robot.commands.Intake.MoveIntake;
import frc.robot.commands.Intake.IntakeStow;
import frc.robot.commands.Shooter.ShooterRev;
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
    private final Joystick driver2 = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
        // Swerve
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

        // Intake + Shoot
    private final JoystickButton dropAndIntake = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton aimAndShoot = new JoystickButton(driver, XboxController.Button.kX.value);
        // TODO: Intake from driver station


    /* Co-driver Buttons */
    private final int armAxis = XboxController.Axis.kRightY.value;
    private final int climbAxis = XboxController.Axis.kLeftY.value;
    private final JoystickButton aim = new JoystickButton(driver2, XboxController.Button.kA.value);

        // Intake (Safety measures if something has gone wrong with the game piece)
    private final JoystickButton intakeIn = new JoystickButton(driver2, XboxController.Button.kA.value);
    private final JoystickButton intakeOut = new JoystickButton(driver2, XboxController.Button.kLeftBumper.value);


    private final JoystickButton spinShooter = new JoystickButton(driver2, XboxController.Button.kA.value);
    


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Intake s_Intake = new Intake();
    private final Climber s_Climber = new Climber();

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
        dropAndIntake.toggleOnTrue(new IntakeIn(s_Intake));

        /* Co-Driver manual Commands */
        spinShooter.toggleOnTrue(new ShooterRev(s_Intake));

        //(new MoveIntake(s_Intake, () -> -driver.getRawAxis(armAxis));
        //s_Climber.setDefaultCommand(new MoveClimber(s_Climber, () -> -driver.getRawAxis(climbAxis)));


        //aim.whileTrue(new Aim(s_Swerve, s_Intake));

        // dropIntake.whileTrue(Commands.parallel(
        //         new IntakeDeploy(s_Intake),
        //         new IntakeIn(s_Intake)))
        //         .onFalse(new IntakeStow(s_Intake));
        
        // aimAndShoot.whileTrue(Commands.parallel(
        //     new IntakeUp(s_Intake), 
        //     new ShooterRev(s_Intake)))
        //     .onFalse(new IntakeStow(s_Intake));

        
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
