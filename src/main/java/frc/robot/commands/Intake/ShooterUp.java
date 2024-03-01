package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.ShooterMovement;

public class ShooterUp extends Command {  
    
    private Pivot s_Pivot;
    private ShooterMovement Movement;

    public ShooterUp(Pivot s_Shooter, ShooterMovement Movement)
    {
        this.s_Pivot = s_Shooter;
        this.Movement = Movement;
    }

    public void execute() 
    {
        s_Pivot.setShooterPivotSpeed(Movement);
    }
}
