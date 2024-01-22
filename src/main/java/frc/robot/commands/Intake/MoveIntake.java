package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;


public class MoveIntake extends Command {
    Intake s_Intake;
    private DoubleSupplier speed;

    public MoveIntake(Intake s_Intake, DoubleSupplier speedSupplier) {
        this.s_Intake = s_Intake;
        //addRequirements(s_Intake);
        this.speed = speedSupplier;
        //addRequirements(s_Intake);
    }

    
    @Override
    public void execute() {
        System.out.println(MathUtil.applyDeadband(speed.getAsDouble(), Constants.stickDeadband));
        s_Intake.setArmMotorSpeeds(MathUtil.applyDeadband(speed.getAsDouble(), Constants.stickDeadband));
    }
}
