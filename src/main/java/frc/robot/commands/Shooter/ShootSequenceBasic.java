// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.IntakeStatus;
import frc.robot.subsystems.Shooter.ShootPosition;
import frc.robot.subsystems.Shooter.ShooterState;

/**
 * Automated control sequence to spin-up shooter, eject note, then spin-down
 * shooter.
 * 
 * @author 5985
 * @author Aidan
 */
public class ShootSequenceBasic extends Command {
    /**
     * MAX Total time allowed for the shooter to spin up and shoot if the beam break
     * doesn't work
     */
    private double SHOOT_TIME = 1;
    private double EJECT_TIME = 5;
    private double EJECT_DELAY = 0.02;
    private boolean EJECTED = false;

    /**
     * Total time allowed for the shooter to spin up. The difference between the two
     * values will be the time allowed for note to eject. This is only used when we
     * are not using vision alignment.
     */
    private double SHOOT_SPIN_UP_TIME = 0;

    Shooter s_Shooter;
    Intake s_Intake;
    Swerve s_Swerve;
    Timer m_timer = new Timer();

    public ShootSequenceBasic(Shooter s_Shooter, Intake s_Intake, Swerve s_Swerve) {
        this.s_Shooter = s_Shooter;
        this.s_Intake = s_Intake;
        this.s_Swerve = s_Swerve;

        //addRequirements(s_Shooter);
    }

    /**
     * Called when the command is initially scheduled.
     * Sets appropriate values for time limits and directions based on current
     * shooter position
     * 
     * @author 5985
     * @author Aidan
     */
    @Override
    public void initialize() {
        EJECTED = false;
        if (s_Shooter.getShootPosition() == ShootPosition.SPEAKER) {
            // speaker shot
            // s_Intake.setIntakeStatus(IntakeStatus.IN);
            s_Shooter.setShooterState(ShooterState.RUNNING);

            // Total time allowed for the shooter to spin up and shoot
            if (!s_Swerve.getVisionAlignmentBool()) {
                SHOOT_TIME = 2; // seconds
                SHOOT_SPIN_UP_TIME = 1; // seconds
            } else {
                SHOOT_TIME = 1; // seconds
                SHOOT_SPIN_UP_TIME = 0; // seconds
            }
            
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
                EJECT_TIME = m_timer.get();
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
     * Returns true (Finished) when shooter has had enough time to eject note, or
     * note is no-longer detected in intake
     * 
     * @author 5985
     * @author Aidan
     */
    @Override
    public boolean isFinished() 
    {
        if (s_Intake.getBeamBreak()) 
        {
            if (EJECTED && m_timer.hasElapsed(EJECT_TIME + EJECT_DELAY)) 
            {
                EJECTED = false;
                return true;
            }
            else
            {
                EJECTED = true;
                EJECT_TIME = m_timer.get();
            }
        }
        if (m_timer.hasElapsed(SHOOT_TIME)) 
        {
            return true;
        } 
        else 
        {
            return false;
        }
        
    }
}