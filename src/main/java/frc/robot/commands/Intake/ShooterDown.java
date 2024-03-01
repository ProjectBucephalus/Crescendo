package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.ShooterMovement;

public class ShooterDown extends Command{

    private Pivot s_Pivot;
    private ShooterMovement Movement;
    private DoubleSupplier manualSpeedAxis;

    public ShooterDown(Pivot s_Shooter, DoubleSupplier speedAxis)
    {
        this.s_Pivot = s_Shooter;
        manualSpeedAxis = speedAxis;
    }

    public void execute() 
    {
        s_Pivot.setShooterPivotSpeed(manualSpeedAxis.getAsDouble());
    }
}

//new ShooterDown(s_Shooter, () -> axisthing.getAsDouble())