package frc.robot;

import java.util.Arrays;
import java.util.List;

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
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.VisionCommands.AimToSpeakerNoDrive;
import frc.robot.VisionCommands.TurnToNote;
import frc.robot.VisionCommands.aimToSpeakerSequence;
import frc.robot.commands.GetMulitNote;
import frc.robot.commands.PointAndPathFindCommand;
import frc.robot.commands.PointToAngle;
import frc.robot.commands.StabiliserBar;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.BuddyClimb.DeployBuddyClimber;
import frc.robot.commands.BuddyClimb.StopBuddyClimber;
import frc.robot.commands.Climber.*;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Shooter.*;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteVision;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.PivotPosition;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Climber.ClimberPosition;
import frc.robot.subsystems.Intake.IndexerState;
import frc.robot.subsystems.Intake.IntakeStatus;
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
    private final Pivot s_Pivot = new Pivot(s_Swerve);
    private final Climber s_Climber = new Climber();
    private final Shooter s_Shooter = new Shooter();
    private final NoteVision s_NoteVision = new NoteVision(s_Swerve);

    private final SendableChooser<String> m_chosenAuto = new SendableChooser<>();
    private final SendableChooser<Pose2d> m_startLocation = new SendableChooser<>();

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    public String selectedAuto = "";

    // We use these to check if the robots starting pose has changed in robot disabled periodic and update the starting pos odometry.
    private Pose2d m_prevInitialPose = new Pose2d();

    /* Autonomous */
    // private final SendableChooser<Command> autoChooser;
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

        NamedCommands.registerCommand("IntakeAndDeployPivot", new IntakeAndDeployPivot(s_Pivot, s_Intake, null));
        NamedCommands.registerCommand("StopIntakeAndStow", new StopIntakeAndStow(s_Pivot, s_Intake));
        NamedCommands.registerCommand("AimToSpeaker", new AimToSpeakerNoDrive(s_Swerve, s_Pivot));
        NamedCommands.registerCommand("Start Shooter", new ShooterRev(s_Shooter));
        NamedCommands.registerCommand("Stop Shooter", new ShooterIdle(s_Shooter));
        NamedCommands.registerCommand("Intake Suck", new IntakeSuck(s_Intake));
        NamedCommands.registerCommand("Intake Stop", new IntakeStop(s_Intake));
        NamedCommands.registerCommand("Move to Base Speaker Angle",
                new MovePivotToPosition(s_Pivot, PivotPosition.SPEAKER_MANUAL));
        NamedCommands.registerCommand("Stow Pivot", new MovePivotToPosition(s_Pivot, PivotPosition.STOWED));

        configureButtonBindings();
        configureAutos();

        // NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        // NamedCommands.registerCommand("print hello", Commands.print("hello"));

        autoChooser = AutoBuilder.buildAutoChooser();
        // SmartDashboard.putData("Auto Chooser", autoChooser);

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
        driver.back()          .onTrue(new ShootSequence(s_Shooter, s_Intake, s_Swerve));
        // made this the same as the robot centric so they act as one func
        driver.leftTrigger()   .whileTrue(new TurnToNote(s_Swerve, s_NoteVision, () -> -driver.getRawAxis(translationAxis), () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(BRAKE_AXIS)));

        /* Pass in codriver for controller to receive rumble */
        driver.leftBumper()    .whileTrue(new aimToSpeakerSequence(s_Swerve,s_Shooter,s_Pivot, coDriver.getHID(), () -> -driver.getRawAxis(translationAxis), () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(BRAKE_AXIS)));
       
        /* Pass in driver for controller to receive rumble when note in intake*/
        driver.rightBumper()   .whileTrue(new IntakeAndDeployPivot(s_Pivot, s_Intake, driver.getHID())) .onFalse(new StopIntakeAndStow(s_Pivot, s_Intake));

        driver.povUp()         .onTrue(new UnlockClimber(s_Climber));
        driver.povDown()       .onTrue(new LockClimber(s_Climber));
       
        driver.y()             .whileTrue(new PointAndPathFindCommand(s_Swerve, FieldConstants.AMP, PathPlannerPath.fromPathFile("Line Up With Amp"), () -> -driver.getRawAxis(translationAxis), () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(rotationAxis)));
        driver.x()             .whileTrue(new PointAndPathFindCommand(s_Swerve, FieldConstants.RIGHT_STAGE, PathPlannerPath.fromPathFile("Line Up With Right Stage"), () -> -driver.getRawAxis(translationAxis), () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(rotationAxis)));
        driver.b()             .whileTrue(new PointAndPathFindCommand(s_Swerve, FieldConstants.LEFT_STAGE, PathPlannerPath.fromPathFile("Line Up With Left Stage"), () -> -driver.getRawAxis(translationAxis), () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(rotationAxis)));
        driver.a()             .whileTrue(new PointAndPathFindCommand(s_Swerve, FieldConstants.BACK_STAGE, PathPlannerPath.fromPathFile("Line Up With Back Stage"), () -> -driver.getRawAxis(translationAxis), () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(rotationAxis)));
        
        
        /* Co-Driver Buttons */

        // coDriver.leftTrigger() .onTrue(new ShooterFeed(s_Intake)).onFalse(new ShooterIdle(s_Shooter).alongWith(new InstantCommand(()->s_Intake.setIntakeStatus(IntakeStatus.STOPPED))));
        coDriver.leftTrigger() .onTrue(new ShootSequence(s_Shooter, s_Intake, s_Swerve));
        coDriver.leftBumper()  .onTrue(new ShooterRev(s_Shooter)); 
        coDriver.rightTrigger().onTrue(new IntakeSuck(s_Intake).andThen(new InstantCommand(()->s_Intake.setIndexerState(IndexerState.OUT)))).onFalse(new IntakeStop(s_Intake).andThen(new InstantCommand(()->s_Intake.setIndexerState(IndexerState.STOPPED)))); //Indexer out.
        coDriver.rightBumper() .onTrue(new IntakeSpit(s_Intake).alongWith(new InstantCommand(()->s_Shooter.setShooterState(ShooterState.OUT)))).onFalse(new IntakeStop(s_Intake).alongWith(new InstantCommand(()->s_Shooter.setShooterState(ShooterState.IDLE))));

        coDriver.x()           .onTrue(new MovePivotToPosition(s_Pivot, PivotPosition.DEPLOYED));
        coDriver.y()           .onTrue(new MovePivotToPosition(s_Pivot, PivotPosition.AMP)); //speaker base
        coDriver.a()           .onTrue(new MovePivotToPosition(s_Pivot, PivotPosition.STOWED));
        //coDriver.y()           .onTrue(new MovePivotToPosition(s_Pivot, PivotPosition.SPEAKER));

        coDriver.povRight()    .onTrue(new StabiliserBar(s_Intake, StabiliserPos.IN)).onFalse(new StabiliserBar(s_Intake, StabiliserPos.STOPPED));
        coDriver.povLeft()     .onTrue(new StabiliserBar(s_Intake, StabiliserPos.OUT)).onFalse(new StabiliserBar(s_Intake, StabiliserPos.STOPPED));
        
        coDriver.povDown()     .onTrue(new DeployBuddyClimber(s_Climber)).onFalse(new StopBuddyClimber(s_Climber));

        // coDriver.povUp()       .onTrue(new ClimberExtend(s_Climber));
        // coDriver.povDown()     .onTrue(new ClimberRetract(s_Climber));
        coDriver.back()        .onTrue(new InstantCommand(() -> s_Climber.setClimberPosition(ClimberPosition.STOPPED)));
        SmartDashboard.putData("Trigger Shot", (new ShootSequence(s_Shooter, s_Intake, s_Swerve)));
        
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
                            2.5, 2,
                            Units.degreesToRadians(360), Units.degreesToRadians(540)),
                    new GoalEndState(0.0, endPos.getRotation()));

            // Prevent this path from being flipped on the red alliance, since the given
            // positions are already correct
            path.preventFlipping = true;

            AutoBuilder.followPath(path).schedule();
        }));

    }
    private void configureAutos() {
        // List of start locations
        List<String> autonamesDropdown = Arrays.asList("S1-S2", "S3-S2", "S1-S2-S3", "S3-S2-S1", "S2-S1", "S1-C1", "C4", "C5", "S3-C4-C5", "W" );

        m_startLocation.setDefaultOption("NotAmp Side", FieldConstants.ROBOT_START_1);
        m_startLocation.addOption("Center", FieldConstants.ROBOT_START_2);
        m_startLocation.addOption("Amp Side", FieldConstants.ROBOT_START_3);

        for (String autoNm : autonamesDropdown) {
            m_chosenAuto.addOption(autoNm, autoNm);
        }

        m_chosenAuto.setDefaultOption("S1-S2", "S1-S2");

        m_startLocation.setDefaultOption("NotAmp Side", FieldConstants.ROBOT_START_1);

        SmartDashboard.putData("Start Location", m_startLocation);

        SmartDashboard.putData("m_chosenAuto", m_chosenAuto);

        SmartDashboard.putString("Auto Chooser", selectedAuto);

    }

    public Pose2d getInitialPose() {
        return FieldConstants.flipPose(m_startLocation.getSelected());
    }

    public String getAutoPopulator() {
        return m_chosenAuto.getSelected();
    }

    public boolean autoHasChanged() {
        Pose2d initialPose = getInitialPose();
        // We don't compare poses with "==". That compares object IDs, not value.
        boolean changed = !initialPose.equals(m_prevInitialPose);
        m_prevInitialPose = initialPose;
        return changed;
    }

    public Swerve getSwerve() {
        return s_Swerve;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand() {
        var transforms = FieldConstants.buildNoteList(SmartDashboard.getString("Auto Chooser", "W"));
        return new GetMulitNote(transforms, s_Swerve, s_NoteVision, s_Shooter, s_Pivot, s_Intake, s_Climber);
    }

}
