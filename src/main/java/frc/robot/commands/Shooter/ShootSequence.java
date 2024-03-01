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

/**
 * Automated control sequence to spin-up shooter, eject note, then spin-down shooter.
 * @author 5985
 * @author Aidan
 */
public class ShootSequence extends Command {
    /** MAX Total time allowed for the shooter to spin up and shoot if the beam break doesn't work*/
    private double SHOOT_TIME;

    /**
     * Total time allowed for the shooter to spin up. The difference between the two 
     * values will be the time allowed for note to eject.
     */
    private double SHOOT_SPIN_UP_TIME;

    Shooter s_Shooter;
    Intake s_Intake;
    Timer m_timer = new Timer();

    public ShootSequence(Shooter s_Shooter, Intake s_Intake) {
        this.s_Shooter = s_Shooter;
        this.s_Intake = s_Intake;

        addRequirements(s_Shooter);
    }

    /**
     * Called when the command is initially scheduled. 
     * Sets appropriate values for time limits and directions based on current shooter position
     * @author 5985
     * @author Aidan
     */
    @Override
    public void initialize() {
        if (s_Shooter.getShootPosition() == ShootPosition.SPEAKER) {
            // speaker shot
            // s_Intake.setIntakeStatus(IntakeStatus.IN);
            s_Shooter.setShooterState(ShooterState.RUNNING);

            // Total time allowed for the shooter to spin up and shoot
            SHOOT_TIME = 2; // seconds
            SHOOT_SPIN_UP_TIME = 1.3; // seconds
        } else if (s_Shooter.getShootPosition() == ShootPosition.AMP) {
            // amp shot
            SHOOT_TIME = 1.5; // seconds
            SHOOT_SPIN_UP_TIME = 0; // seconds
            s_Intake.setIntakeStatus(IntakeStatus.OUT);
        } else if (s_Shooter.getShootPosition() == ShootPosition.TRAP) {
            // amp shot
            s_Shooter.setShooterState(ShooterState.RUNNING);
        }

        m_timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    /**
     * Sets intake feed direction only after shooter has had enough time to spin-up
     */
    @Override
    public void execute() {
        if (m_timer.hasElapsed(SHOOT_SPIN_UP_TIME)) {
            if (s_Shooter.getShootPosition() == ShootPosition.SPEAKER) {
                /* Speaker Shot */
                s_Intake.setIntakeStatus(IntakeStatus.IN_FOR_SHOOTING);
                // s_Shooter.setShooterState(ShooterState.RUNNING);
            } else if (s_Shooter.getShootPosition() == ShootPosition.AMP) {
                /* Amp Shot */
                s_Intake.setIntakeStatus(IntakeStatus.OUT);
            } else if (s_Shooter.getShootPosition() == ShootPosition.TRAP) {
                /* Trap Shot */
                s_Intake.setIntakeStatus(IntakeStatus.IN_FOR_SHOOTING);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_Shooter.setShooterState(ShooterState.IDLE);
        s_Intake.setIntakeStatus(IntakeStatus.STOPPED);
    }

    // Returns true when the command should end.
    /**
     * Returns true (Finished) when shooter has had enough time to eject note, or note is no-longer detected in intake
     * @author 5985
     * @author Aidan
     */
    @Override
    public boolean isFinished() {
        return (m_timer.hasElapsed(SHOOT_TIME) || s_Intake.getBeamBreak()) ;
    }
}