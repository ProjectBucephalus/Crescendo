package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.lib.math.Conversions;
import frc.robot.CTREConfigs;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Shooter extends SubsystemBase {
    // motors
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);


    public TalonFX mTopShooter = new TalonFX(Constants.Shooter.mTopShooterID);
    public TalonFX mBottomShooter = new TalonFX(Constants.Shooter.mBottomShooterID);

    private ShootPosition shooterMode = ShootPosition.SPEAKER;

    public enum FlapPosition {
        OPEN,
        CLOSED,
    };

    public enum ShooterState {
        RUNNING,
        STOPPED,
        IDLE,
    };

    public enum ShootPosition {
        AMP,
        SPEAKER,
        TRAP,
    };

    public Shooter() {
        
        
    }
    
    /* sets shooter to full speed */
    public void setShooterState(ShooterState state) {
        // double bottomSpeed = SmartDashboard.getNumber("bottomShooterSpeed", 1);
        // double topSpeed = SmartDashboard.getNumber("topShooterSpeed", 1);
        SmartDashboard.putString("Current State of Shooter Motors for sim", state.name());
        switch (state) {
            case RUNNING:
                driveDutyCycle.Output = 0.5;
                mBottomShooter.setControl(driveDutyCycle);                

                driveDutyCycle.Output = 0.5;
                mTopShooter.setControl(driveDutyCycle);
                break;
            case STOPPED:
                driveDutyCycle.Output = 0;
                mBottomShooter.setControl(driveDutyCycle);

                driveDutyCycle.Output = 0;
                mTopShooter.setControl(driveDutyCycle);
            case IDLE:
                driveDutyCycle.Output = 0.2;
                mBottomShooter.setControl(driveDutyCycle);

                driveDutyCycle.Output = 0.2;
                mTopShooter.setControl(driveDutyCycle);
            default:
                break;
        }        
        // SmartDashboard.putNumber("bottomShooterSpeed", bottomSpeed);
        // SmartDashboard.putNumber("topShooterSpeed", topSpeed);
    }

    /* stops shooter */
    // public void stopShooter() {
    //     mTopShooter.set(0);
    //     mBottomShooter.set(0);
    // }

    /* sets shooter to idle speed */
    // public void idleShooter() {
    //     mTopShooter.set(Constants.Shooter.shooterIdleSpeed);
    //     mBottomShooter.set(Constants.Shooter.shooterIdleSpeed);
    // }

    public void setShooterPosition(ShootPosition pos) {
        shooterMode = pos;
    }

    public ShootPosition getShootPosition() {
        return shooterMode;
    }

    public boolean rpmWithinTolerance() {
        return true; // TODO
    }
    @Override
    public void periodic() {
        SmartDashboard.putString("Where am I shooting", getShootPosition().name());
        SmartDashboard.putString("Current State of Motors for sim", getShootPosition().name());
    }
}