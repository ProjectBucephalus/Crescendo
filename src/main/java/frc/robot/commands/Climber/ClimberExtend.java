package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberPosition;

/**
 * Climber extension command
 * 
 * @author 5985
 */
public class ClimberExtend extends Command {

    public Climber s_Climber;

    public ClimberExtend(Climber s_Climber) {
        this.s_Climber = s_Climber;
        addRequirements(s_Climber);
    }

    public void initialize() {

    }

    public void execute() {
        s_Climber.setClimberPosition(ClimberPosition.UP);
    }

    public void end() {
    }

    public boolean isFinished() {
        return true;
        // return s_Climber.getRightLimit() && s_Climber.getLeftLimit();
    }
}
