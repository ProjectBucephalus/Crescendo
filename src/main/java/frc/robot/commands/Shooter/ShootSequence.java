// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeStatus;
import frc.robot.subsystems.Shooter.ShootPosition;
import frc.robot.subsystems.Shooter.ShooterState;

public class ShootSequence extends Command {
    private static final double SHOOT_TIME = 2.7; // seconds
    private static final double SHOOT_SPIN_UP_TIME = 2.0; // seconds

    Shooter s_Shooter;
    Intake s_Intake;
    Timer m_timer = new Timer();

    /** Creates a new TriggerShot. */
    public ShootSequence(Shooter s_Shooter, Intake s_Intake) {
        this.s_Shooter = s_Shooter;
        this.s_Intake = s_Intake;

        addRequirements(s_Shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (s_Shooter.getShootPosition() == ShootPosition.SPEAKER) {
            // speaker shot
            //s_Intake.setIntakeStatus(IntakeStatus.IN);
            s_Shooter.setShooterState(ShooterState.RUNNING);
        } else if (s_Shooter.getShootPosition() == ShootPosition.AMP) {
            // amp shot
            s_Intake.setIntakeStatus(IntakeStatus.OUT);
        } else if (s_Shooter.getShootPosition() == ShootPosition.TRAP) {
            // amp shot
            s_Shooter.setShooterState(ShooterState.RUNNING);
        }

        m_timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_timer.hasElapsed(SHOOT_SPIN_UP_TIME)) {
            if (s_Shooter.getShootPosition() == ShootPosition.SPEAKER) {
                // speaker shot
                s_Intake.setIntakeStatus(IntakeStatus.IN_FOR_SHOOTING);
                //s_Shooter.setShooterState(ShooterState.RUNNING);
            } else if (s_Shooter.getShootPosition() == ShootPosition.AMP){
                // amp shot
                s_Intake.setIntakeStatus(IntakeStatus.OUT);
            } else if (s_Shooter.getShootPosition() == ShootPosition.TRAP){
                // amp shot
                s_Shooter.setShooterState(ShooterState.RUNNING);
            }
        }
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        s_Shooter.setShooterState(ShooterState.STOPPED);
        s_Intake.setIntakeStatus(IntakeStatus.STOPPED);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(SHOOT_TIME);
    }
}