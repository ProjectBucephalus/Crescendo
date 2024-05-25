package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.subsystems.RumbleController;
import frc.robot.subsystems.Swerve;

public class LeftStage extends SequentialCommandGroup
{   
    Swerve s_Swerve;
    RumbleController s_RumbleController;


    public LeftStage(Swerve s_Swerve, RumbleController s_RumbleController)
    {
        this.s_Swerve = s_Swerve;
        this.s_RumbleController = s_RumbleController;

        if (FieldConstants.isRedAlliance()) 
        {
            addCommands
            (
                new PointAndPathFindCommand(s_Swerve, FieldConstants.SOURCE_STAGE, PathPlannerPath.fromPathFile("Line Up With Right Stage"), s_RumbleController)
            );
        }
        else
        {
            addCommands
            (
                new PointAndPathFindCommand(s_Swerve, FieldConstants.AMP_STAGE, PathPlannerPath.fromPathFile("Line Up With Left Stage"), s_RumbleController)
            );
        }
       
    }
}
