package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;


/**
 * Move intake command. 
 * Sends modified axis to intake pivot motor power directly. 
 * Should only be used if sensor-based angle fails.
 * @author 5985
 * @author Alec
 */
public class MovePivot extends Command {
    Pivot s_Pivot;
    private DoubleSupplier speed;
    private boolean manualActive = false;

    /**
     * 
     * @param s_Intake a reference to the intake subsystem
     * @param speedSupplier double supplier speed 
     */
    public MovePivot(Pivot s_Pivot, DoubleSupplier speedSupplier) {
        this.s_Pivot = s_Pivot;
        this.speed = speedSupplier;
        addRequirements(s_Pivot);
    }

    
    /**
     * Sends modified axis value to pivot motor speeds
     * @author 5985
     * @author Alec
     */
    @Override
    public void execute() 
    {
        //System.out.println(MathUtil.applyDeadband(speed.getAsDouble(), Constants.stickDeadband));
        
        if ( Math.abs(MathUtil.applyDeadband(speed.getAsDouble(), Constants.stickDeadband)) >= 0.1 )
        {
            s_Pivot.setArmMotorSpeeds(Constants.Intake.pivotManualGain * MathUtil.applyDeadband(speed.getAsDouble(), Constants.stickDeadband));
            manualActive = true;
        }
        else if (manualActive)
        {
            s_Pivot.setArmMotorSpeeds(0);
            manualActive = false;
        }
    }
}
