package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

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
import frc.robot.Utilities.Limelight;
import frc.robot.VisionCommands.AimToSpeakerNoDrive;
import frc.robot.VisionCommands.aimToSpeaker;
import frc.robot.VisionCommands.aimToSpeakerSequence;
import frc.robot.commands.PointToAngle;
import frc.robot.commands.StabiliserBar;
import frc.robot.commands.TeleopSwerve;
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
import frc.robot.commands.Intake.MovePivot;
import frc.robot.commands.Intake.MovePivotToPosition;
import frc.robot.commands.Intake.StopIntakeAndStow;
import frc.robot.commands.Shooter.ShootSequence;
import frc.robot.commands.Shooter.ShooterIdle;
import frc.robot.commands.Shooter.ShooterRev;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteVision;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.PivotPosition;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.IndexerPosition;
import frc.robot.subsystems.Intake.StabiliserPos;

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
    private final int MANUAL_SHOOTER_AXIS = XboxController.Axis.kRightY.value;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Intake s_Intake = new Intake();
    private final Pivot s_Pivot = new Pivot();
    private final Climber s_Climber = new Climber();
    private final Shooter s_Shooter = new Shooter();
    //private final NoteVision s_NoteVision = new NoteVision();

    private final Limelight m_lime = new Limelight("limelight");

    private final SendableChooser<Command> m_chosenAuto = new SendableChooser<>();
    private final SendableChooser<Pose2d> m_startLocation = new SendableChooser<>();

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
                        () -> driver.leftTrigger().getAsBoolean(),
                        () -> -driver.getRawAxis(BRAKE_AXIS)));

        // s_Vision.setDefaultCommand(new multiTagPoseEstimatior(s_Vision));
        s_Pivot.setDefaultCommand(new MovePivot(s_Pivot, () -> -coDriver.getRawAxis(MANUAL_SHOOTER_AXIS)));
        s_Climber.setDefaultCommand(new MoveClimber(s_Climber, () -> coDriver.getRawAxis(MANUAL_CLIMB_AXIS)));

        configureButtonBindings();

        NamedCommands.registerCommand("IntakeAndDeployPivot", new IntakeAndDeployPivot(s_Pivot, s_Intake, driver.getHID()));
        NamedCommands.registerCommand("StopIntakeAndStow", new StopIntakeAndStow(s_Pivot, s_Intake));
        NamedCommands.registerCommand("AimToSpeaker", new AimToSpeakerNoDrive(s_Swerve, s_Pivot));
        NamedCommands.registerCommand("Start Shooter", new ShooterRev(s_Shooter));
        NamedCommands.registerCommand("Stop Shooter", new ShooterIdle(s_Shooter));
        NamedCommands.registerCommand("Intake Suck", new IntakeSuck(s_Intake));
        NamedCommands.registerCommand("Intake Stop", new IntakeStop(s_Intake));
        NamedCommands.registerCommand("Move to Base Speaker Angle",
                new MovePivotToPosition(s_Pivot, PivotPosition.SPEAKER_MANUAL));
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

        //driver.back()          .onTrue(new InstantCommand(s_Swerve::lockWheels, s_Swerve)); // TODO 
        driver.start()         .onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        //driver.leftTrigger()   .whileTrue(new TurnToNote(s_Swerve, s_NoteVision, () -> -driver.getRawAxis(translationAxis), () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(BRAKE_AXIS)));

        /* Pass in codriver for controller to receive rumble */
        driver.leftBumper()    .whileTrue(new aimToSpeakerSequence(s_Swerve,s_Shooter,s_Pivot, coDriver.getHID(), () -> -driver.getRawAxis(translationAxis), () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(BRAKE_AXIS)));
       
        /* Pass in driver for controller to receive rumble when note in intake*/
        driver.rightBumper()   .whileTrue(new IntakeAndDeployPivot(s_Pivot, s_Intake, driver.getHID())).onFalse(new StopIntakeAndStow(s_Pivot, s_Intake).andThen(new MovePivotToPosition(s_Pivot, PivotPosition.STOWED)));
        driver.povUp()         .onTrue(new UnlockClimber());
        driver.povDown()       .onTrue(new LockClimber(s_Climber));
        driver.povRight()      .onTrue(new StabiliserBar(s_Intake, StabiliserPos.IN)).onFalse(new StabiliserBar(s_Intake, StabiliserPos.STOPPED));
        driver.povLeft()       .onTrue(new StabiliserBar(s_Intake, StabiliserPos.OUT)).onFalse(new StabiliserBar(s_Intake, StabiliserPos.STOPPED));
        // driver.y()             .onTrue(new PointToAngle(s_Swerve, 180));
        // driver.x()             .onTrue(new PointToAngle(s_Swerve, 60));
        // driver.b()             .onTrue(new PointToAngle(s_Swerve, -60));
        // driver.a()             .onTrue(new PointToAngle(s_Swerve, 90));
        // driver.a()             .whileTrue({
            
            
        //     Pose2d endPos = new Pose2d((new Translation2d(2.0, 2.0)), new Rotation2d(Units.degreesToRadians(90)));
        //     // We use pathfind to pose flipped so that it flips correctly to the red alliance.
        //     AutoBuilder.pathfindToPoseFlipped(endPos, new PathConstraints(
        //         4.0,
        //         4.0,
        //         Units.degreesToRadians(360),
        //         Units.degreesToRadians(540)))
        //         .schedule();
        // })););
        
        /* Co-Driver Buttons */

        coDriver               .leftTrigger() .onTrue(new ShootSequence(s_Shooter, s_Intake));
        //coDriver.leftBumper()  .whileTrue(new InstantCommand(() -> s_Intake.setIndexPosition(IndexerPosition.IN))).onFalse(getAutonomousCommand());
        coDriver               .rightTrigger().onTrue(new IntakeSuck(s_Intake)).onFalse(new IntakeStop(s_Intake));
        coDriver               .rightBumper() .whileTrue(new IntakeSpit(s_Intake));

        coDriver.x()           .onTrue(new MovePivotToPosition(s_Pivot, PivotPosition.DEPLOYED));
        coDriver.y()           .onTrue(new MovePivotToPosition(s_Pivot, PivotPosition.AMP));
        coDriver.a()           .onTrue(new MovePivotToPosition(s_Pivot, PivotPosition.STOWED));
        //coDriver.y()           .onTrue(new MovePivotToPosition(s_Pivot, PivotPosition.SPEAKER));
        
        
        coDriver.povLeft()     .whileTrue(new DeployBuddyClimber(s_Climber)).onFalse(new StopBuddyClimber(s_Climber));
        // coDriver.povDown()     .onTrue(new ClimberRetract(s_Climber));
        // coDriver.povUp()       .onTrue(new ClimberExtend(s_Climber));
        
        SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
            Pose2d currentPose = s_Swerve.getEstimatedPose();

            // The rotation component in these poses represents the direction of travel
            //Pose2d startPos = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());
            Pose2d startPos = currentPose;
            Pose2d endPos = new Pose2d((new Translation2d(2.0, 2.0)), new Rotation2d(Units.degreesToRadians(90)));
            //Pose2d endPos = 

            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
            PathPlannerPath path = new PathPlannerPath(
                    bezierPoints,
                    new PathConstraints(
                            4.0, 4.0,
                            Units.degreesToRadians(360), Units.degreesToRadians(540)),
                    new GoalEndState(0.0, endPos.getRotation()));

            // Prevent this path from being flipped on the red alliance, since the given
            // positions are already correct
            path.preventFlipping = true;

            AutoBuilder.followPath(path).schedule();
        }));

    }
    // private void configureAutos() {
    //     // List of start locations
    //     m_startLocation.setDefaultOption("NotAmp Side", FieldConstants.ROBOT_START_1);
    //     m_startLocation.addOption("Center", FieldConstants.ROBOT_START_2);
    //     m_startLocation.addOption("Amp Side", FieldConstants.ROBOT_START_3);
    //     SmartDashboard.putData("Start Location", m_startLocation);

    //     String autoName = "C1-C2";
    //     autoChooser.setDefaultOption(autoName, new GetMultiNoteGeneric(autoName, m_driveTrain, m_noteVision, m_shooter, m_intake));

    //     autoName = "C2-C1";
    //     autoChooser.addOption(autoName, new GetMultiNoteGeneric(autoName, m_driveTrain, m_noteVision, m_shooter, m_intake));

    //     List<String> autonamesDropdown = Arrays.asList("S1-S2", "S1-W-S2", "S1-W-W-S2", "S3-S2", "S1-S2-S3", "S3-S2-S1", "S2-S1", "S1-C1", "C4", "C5", "S3-C4-C5" );

    //     for (String autoNm : autonamesDropdown) {
    //         m_chosenAuto.addOption(autoNm, new GetMultiNoteGeneric(autoNm, m_driveTrain, m_noteVision, m_shooter, m_intake));
    //     }
        
    //     m_chosenAuto.addOption("Test Auto", new NoteAuto(m_driveTrain));
    //     SmartDashboard.putData("Chosen Auto", m_chosenAuto);
    // }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
