// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.VisionCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.commands.CheckPrepStatsAndRumble;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter.ShootPosition;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pivot;

public class aimToSpeakerSequence extends ParallelCommandGroup {

    /** Creates a new PrepareSpeakerShot. */
    public aimToSpeakerSequence(Swerve s_Swerve, Shooter s_Shooter, Pivot s_Pivot, XboxController xboxController, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier brakeSup) {

        addCommands(
                // set shoot mode, so that TriggerShot can be a single command/button
                new CheckPrepStatsAndRumble(s_Pivot, s_Shooter, s_Swerve, xboxController),
                new InstantCommand(() -> s_Shooter.setShooterPosition(ShootPosition.SPEAKER)),
                //new ActiveSetShooter(shooter, shooterPivot, this::getShootValues),
                new aimToSpeaker(s_Swerve, translationSup, strafeSup, brakeSup, s_Pivot)
                
        );
        
    }

}