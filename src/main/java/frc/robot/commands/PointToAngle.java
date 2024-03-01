package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Swerve;

/**
 * PointToAngle Command
 * @author Alec
 * @author 5985
 */
public class PointToAngle extends Command {
    private frc.robot.subsystems.Swerve s_Swerve;
    private double targetRotation;

    public PointToAngle(frc.robot.subsystems.Swerve s_Swerve, double targetRotation)
    {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.targetRotation = targetRotation;
    }

    @Override
    public void execute() 
    {
        new Rotation2d(targetRotation);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
