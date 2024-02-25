package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
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
 * 
 * @author 5985
 */
public class Intake extends SubsystemBase {
    public TalonFX mIntake = new TalonFX(Constants.Intake.mIntakeID);
    public TalonFX mIndexer = new TalonFX(Constants.Intake.mIndexerID);
    public VictorSPX mStabilser = new VictorSPX(Constants.Intake.mStabilserID);


    public DigitalInput BeamBreak = new DigitalInput(9);

    public TalonFXConfiguration IndexerFXConfig = new TalonFXConfiguration();

    private boolean beamBreakBool = false;
    private boolean useBeamBreak = false;

    public enum IntakeStatus {
        IN,
        OUT,
        IN_WITH_BEAM_BREAK,
        STOPPED,
        IN_FOR_SHOOTING
    };

    public enum StabiliserPos {
        OUT, 
        IN,
        STOPPED
    };

    public enum IndexerPosition {
        STOPPED,
        IN,
        OUT,
        IN_WITH_BEAM_BREAK,
        IN_FOR_SHOOTING

    };

    public Intake() {

        SmartDashboard.putNumber("topShooterSpeed", 1);
        SmartDashboard.putNumber("bottomShooterSpeed", 1);

        IndexerFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        IndexerFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        mIndexer.getConfigurator().apply(IndexerFXConfig);
        // SmartDashboard.putNumber("pivotPosition", 1.2);
    }

    /**
     * sets the speed of the motor
     * 
     * @param speed the speed that the arm rotates at
     */
    public void setIntakeSpeed(double speed, boolean useBeamBreak) {
        mIntake.set(speed);

    }

    /**
     * sets the speed that the intake motors rotate to suck in notes
     * 
     * @param speed the motor speed of the intake motors
     */
    public void setIntakeStatus(IntakeStatus status) {
        SmartDashboard.putString("intake Status", status.name());
        switch (status) {
            case IN_FOR_SHOOTING:
                setIndexPosition(IndexerPosition.IN_FOR_SHOOTING);
                setIntakeSpeed(1, false);
                useBeamBreak = false;
                break;
            case IN:
                setIndexPosition(IndexerPosition.IN);
                setIntakeSpeed(0.50, false);
                useBeamBreak = false;
                break;
            case OUT:
                setIndexPosition(IndexerPosition.OUT);
                setIntakeSpeed(-0.50, false);
                useBeamBreak = false;
                break;
            case IN_WITH_BEAM_BREAK:
                
                setIndexPosition(IndexerPosition.IN_WITH_BEAM_BREAK);
                setIntakeSpeed(0.50, true);
                useBeamBreak = true;
                break;
            case STOPPED:
                setIndexPosition(IndexerPosition.STOPPED);
                setIntakeSpeed(0, false);
                useBeamBreak = false;
                break;

            default:
                break;
        }
    }

    /**
     * sets the position of the flap
     * @param pos can be open or closed
     */
    public void setIndexPosition(IndexerPosition pos) {
        SmartDashboard.putString("indexer Status", pos.name());
        switch (pos) {
            // this means that we are running the intake in. We need to do logic on what we actually need to do. 
            case IN:
                mIndexer.set(0.25);
                break;
        
            case OUT:
                mIndexer.set(-0.25);
                
                break;
            case STOPPED:
                mIndexer.set(0.0);
                break;
            case IN_WITH_BEAM_BREAK:
                mIndexer.set(-0.25);
                break;
            case IN_FOR_SHOOTING:
                mIndexer.set(1);
                break;
        }
    }

    public void setStabliserPos(StabiliserPos pos) {
        SmartDashboard.putString("Stabliser Statsu", pos.name());
        switch (pos) {
            case IN:
                mStabilser.set(ControlMode.PercentOutput, -0.6);

                break;
            case OUT:
                mStabilser.set(ControlMode.PercentOutput, 0.6);
                break;
            case STOPPED:
                mStabilser.set(ControlMode.PercentOutput, 0);
                break;
            default:
                break;
            
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("BeamBreak", beamBreakBool);
        beamBreakBool = BeamBreak.get();
        if (useBeamBreak) {
            if (!beamBreakBool) {
                setIntakeStatus(IntakeStatus.STOPPED);
            }
        }
    }

}