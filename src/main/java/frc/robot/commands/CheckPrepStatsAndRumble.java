// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pivot;

public class CheckPrepStatsAndRumble extends Command {
    private final Pivot s_Pivot;
    private final Shooter m_Shooter;
    private final Swerve s_Swerve;
    private final XboxController m_controller;

    // Value between 0-1. 0 is nothing 1 is a lot.
    // 0.3 should be nonintrusive to the driver
    private static final double RUMBLE_INTENSITY = 0.1;

    /** Creates a new CheckPrepStatsAndRumble. */
    public CheckPrepStatsAndRumble(Pivot s_Pivot, Shooter s_Shooter, Swerve s_Swerve, XboxController xboxController) {
        this.s_Pivot = s_Pivot;
        this.m_Shooter = s_Shooter;
        this.s_Swerve = s_Swerve;
        this.m_controller = xboxController;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (s_Pivot.angleWithinTolerance() && m_Shooter.rpmWithinTolerance() && s_Swerve.getWithinRequiredHeading()) {
            // Sets rumble
            m_controller.setRumble(RumbleType.kBothRumble, RUMBLE_INTENSITY);
            SmartDashboard.putBoolean("Ready to shoot?", true);
        } else {
            m_controller.setRumble(RumbleType.kBothRumble, 0);
            SmartDashboard.putBoolean("Ready to shoot?", false);
        }


    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Turns it off once done
        m_controller.setRumble(RumbleType.kBothRumble, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}