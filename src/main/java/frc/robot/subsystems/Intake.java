package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Intake.BeamBreakStatus;
import frc.lib.math.Conversions;
import frc.robot.CTREConfigs;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * intake subsystem 
 * @author 5985
*/
public class Intake extends SubsystemBase {
    public TalonFX mIntake = new TalonFX(Constants.Intake.mIntakeID);
    public VictorSPX mFlap = new VictorSPX(Constants.Intake.mFlapID);

    public DigitalInput BeamBreak = new DigitalInput(9);

    private boolean beamBreakBool = false;
    private boolean useBeamBreak = false;
    


    public enum IntakeStatus{
        IN,
        OUT,
        IN_WITH_BEAM_BREAK,
        STOPPED,
    };


    public enum FlapPosition {
        OPEN,
        CLOSED,
    };

    

    public Intake() {

        SmartDashboard.putNumber("topShooterSpeed", 1);
        SmartDashboard.putNumber("bottomShooterSpeed", 1);
        //SmartDashboard.putNumber("pivotPosition", 1.2);
    }

    /**
     * sets the speed of the motor
     * @param speed the speed that the arm rotates at
     */
    public void setIntakeSpeed(double speed, boolean useBeamBreak) {
        this.useBeamBreak = useBeamBreak;
        if (useBeamBreak && !BeamBreak.get())
            mIntake.set(0);
        else
            mIntake.set(-speed);
            
    }

    /**
     * sets the speed that the intake motors rotate to suck in notes
     * @param speed the motor speed of the intake motors
     */
    public void setIntakeStatus(IntakeStatus status) {
        switch (status) {
            case IN:
                setIntakeSpeed(0.25, false);
                useBeamBreak = false;
                break;
            case OUT:
                setIntakeSpeed(-0.25, false);
                useBeamBreak = false;
                break;
            case IN_WITH_BEAM_BREAK:
                setIntakeSpeed(1, true);
                useBeamBreak = true;
                break;
            case STOPPED:
                setIntakeSpeed(0, false);
                useBeamBreak = false;
                break;
        
            default:
                break;
        }
    }

    /**
     * sets the speed that the flap deploys at
     * @param speed the speed that the flap deploys at
     */
    public void setFlapSpeed(double speed) {
        mFlap.set(VictorSPXControlMode.PercentOutput, speed);
    }

    /**
     * sets the position of the flap
     * @param pos can be open or closed
     */
    public void setFlapPosition (FlapPosition pos) {
        switch (pos) {
            case OPEN:
                mFlap.set(ControlMode.PercentOutput, 0.5);
                break;
        
            case CLOSED:
                mFlap.set(ControlMode.PercentOutput, -0.5);
                SmartDashboard.putNumber("Output Voltage", mFlap.getMotorOutputVoltage());
                SmartDashboard.putNumber("Percent Output", mFlap.getMotorOutputPercent()); // prints the percent output of the motor (0.5)
                SmartDashboard.putNumber("Bus voltage", mFlap.getBusVoltage()); // prints the bus voltage seen by the motor controller
                break;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("BeamBreak", beamBreakBool);
        beamBreakBool = BeamBreak.get();
        if (useBeamBreak) {
            if(!beamBreakBool) {
                setIntakeStatus(IntakeStatus.STOPPED);
            }
        }
    }
    
}