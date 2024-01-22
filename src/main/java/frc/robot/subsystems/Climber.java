package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    
    public TalonFX mLeftClimber = new TalonFX(17);
    public TalonFX mRightClimber = new TalonFX(14);

    public void setSpeed(double speed) {
        mLeftClimber.set(speed);
        mRightClimber.set(-speed);
    }
}
