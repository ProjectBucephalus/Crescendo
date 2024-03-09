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
    

    Shooter s_Shooter;
    Intake s_Intake;

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
        switch (s_Shooter.getShootPosition()) 
        {
            case SPEAKER:
                s_Intake.setIntakeStatus(IntakeStatus.IN_FOR_SHOOTING);
                // s_Shooter.setShooterState(ShooterState.RUNNING);
                break;

            case AMP:
                s_Intake.setIntakeStatus(IntakeStatus.OUT);
                break;
            
            case TRAP:
                s_Intake.setIntakeStatus(IntakeStatus.IN_FOR_SHOOTING);
                break;
        
            default:
                break;
        } 
    }

    // Called every time the scheduler runs while the command is scheduled.
    /**
     * Sets intake feed direction only after shooter has had enough time to spin-up
     */
    @Override
    public void execute() {
        switch (s_Shooter.getShootPosition()) 
        {
            case SPEAKER:
                s_Intake.setIntakeStatus(IntakeStatus.IN_FOR_SHOOTING);
                // s_Shooter.setShooterState(ShooterState.RUNNING);
                break;

            case AMP:
                s_Intake.setIntakeStatus(IntakeStatus.OUT);
                break;
            
            case TRAP:
                s_Intake.setIntakeStatus(IntakeStatus.IN_FOR_SHOOTING);
                break;
        
            default:
                break;
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
        return (s_Intake.getBeamBreak()) ;
    }
}