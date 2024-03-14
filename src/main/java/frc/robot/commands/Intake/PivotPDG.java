package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.PivotPosition;

/**
 * Moves the pivot to a set angle using a PDG loop
 * @author 5985
 */
public class PivotPDG extends Command 
{
    public boolean isFinished = false;
    private Pivot s_Pivot;
    private double targetAngle;
    
    /**
    * Moves the pivot to a set angle using a PDG loop
    * @param s_Pivot An instance of the Pivot subsystem
    * @param target The position to move to, as real-world angle (optional)
    * @author 5985
    */
    public PivotPDG(Pivot s_Pivot, double target) 
    {
        this.s_Pivot = s_Pivot;
        this.targetAngle = target;
        addRequirements(s_Pivot);
        SmartDashboard.putString("Pivot PDG Status : ", "Created (a)");
    }

    /**
    * Holds the pivot at the current angle using a PDG loop
    * @param s_Pivot An instance of the Pivot subsystem
    * @author 5985
    */
    public PivotPDG(Pivot s_Pivot) 
    {
        this.s_Pivot = s_Pivot;
        targetAngle = s_Pivot.getPivotPos();
        addRequirements(s_Pivot);
        SmartDashboard.putString("Pivot PDG Status : ", "Created (b)");
    }

    @Override
    public void initialize()
    {   
        // On initialisation, runs the version of pivotPDGCycle that sets the desired position,  
        // using either the current position or an input value as the desired position based on the constructor used
        s_Pivot.pivotPDGCycle(targetAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {   
        // While executing, runs the version of pivotPDGCycle that maintains current position
        s_Pivot.pivotPDGCycle();
    }

    @Override
    public boolean isFinished() 
    {
        return isFinished;
    }

    @Override
    public void end(boolean endstate)
    {
        SmartDashboard.putString("Pivot PDG Status : ", "Ended");
    }
}
