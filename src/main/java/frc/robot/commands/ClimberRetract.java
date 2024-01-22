package frc.robot.commands;
import frc.robot.subsystems.Climber;

public class ClimberRetract {

    public void initialize() {

    }
    
    public void execute()
    {
        Climber.climberDown();
    }

    public void end(boolean stopping) {
        Climber.climberStop();
    }
}
