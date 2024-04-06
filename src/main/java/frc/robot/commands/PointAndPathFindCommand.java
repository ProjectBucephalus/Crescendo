// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.RumbleController;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.RumbleController.Controllers;
import frc.robot.VisionCommands.AlignToTrap;
import frc.robot.subsystems.RumbleController.RumbleStates;

public class PointAndPathFindCommand extends SequentialCommandGroup {
    RumbleController s_RumbleController;

    /**
     * Constructs a new PointAndPathFind command group.
     * 
     * @param s_Swerve       The Swerve subsystem instance.
     * @param targetLocation The target location to align with. From FieldConstants.
     * @param path           The predefined path to follow, represented as a
     *                       PathPlannerPath made in pathplanner.
     */

     
    public PointAndPathFindCommand(Swerve s_Swerve, Transform2d targetLocation, PathPlannerPath path, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotatSup, RumbleController s_RumbleController) {
        this.s_RumbleController = s_RumbleController;
        // Create the constraints to use while pathfinding. The constraints defined in
        PathConstraints constraints = new PathConstraints(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        addCommands
        (
                new InstantCommand(()->s_Swerve.setVisionAlignmentBool(true)),
                new PointToAngle(s_Swerve, targetLocation).withTimeout(0.5),
                AutoBuilder.followPath(path),
                new AlignToTrap(s_Swerve, targetLocation)
        );
        s_RumbleController.setRumbleStatus(RumbleStates.SHOOTREADY, true);

    }
}
