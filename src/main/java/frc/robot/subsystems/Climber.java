package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    public TalonFX mLeftClimber = new TalonFX(Constants.Climber.mLeftClimbID);
    public TalonFX mRightClimber = new TalonFX(Constants.Climber.mRightClimbID);

    public Climber() { 
    }

    public void setSpeed(double speed) {
        mLeftClimber.set(speed);
        mRightClimber.set(-speed);
    }

    public double getPosition() {
        SmartDashboard.putNumber("ClimberPosition", mLeftClimber.getPosition().getValueAsDouble());
        return (mLeftClimber.getPosition().getValueAsDouble());
    }
}
