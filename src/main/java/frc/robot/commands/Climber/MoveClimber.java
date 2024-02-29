package frc.robot.commands.Climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;


/**
 * Climber movement command
 * @author 5985
 */
public class MoveClimber extends Command {

    private Climber climber;
    private DoubleSupplier speed;

    /**
     * Moves the climber
     * @param climber reference to the climber subsystem
     * @param speed_sup double supplier for the climber speed [-1..1]
     */
    public MoveClimber(Climber climber, DoubleSupplier speed_sup) 
    {
        this.climber = climber;
        this.speed = speed_sup;
        addRequirements(climber);
        
    }

    @Override
    public void execute() 
    {
        System.out.println(MathUtil.applyDeadband(speed.getAsDouble(), Constants.stickDeadband));
        climber.setSpeed(MathUtil.applyDeadband(speed.getAsDouble(), Constants.stickDeadband));
    }
    
}
