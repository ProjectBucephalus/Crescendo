package frc.robot.commands;
import frc.robot.subsystems.Climber;

public class ClimberExtend {

    public void initialize() {

    }
    
    public void execute()
    {
        Climber.climberUp();
    }

    public void end(boolean stopping) {
        Climber.climberStop();
    }
}
