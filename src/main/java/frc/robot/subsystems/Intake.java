package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;

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
     * @param speed Intake motor speed [-1..1]
     * @param useBeamBreak (Unused) Set true to stop intake when note is detected
     */
    public void setIntakeSpeed(double speed, boolean useBeamBreak) {
        mIntake.set(speed);

    }

    /**
     * sets the speed that the intake motors rotate to suck in notes
     * 
     * @param status Enum value corresponding to intake motor speed and related values
     */
    public void setIntakeStatus(IntakeStatus status) {
        SmartDashboard.putString("Intake Status", status.name());
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
                setIntakeSpeed(0.75, true);
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
     * sets the status of the indexer 
     * TODO replace values with constants
     * @param pos Enum value corresponding to indexer speeds
     * @author 5985
     * @author Aidan
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
        SmartDashboard.putString("Stabliser Status", pos.name());
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

    public boolean getBeamBreak() {
        return beamBreakBool;
    }

    @Override
    public void periodic() 
    {
        beamBreakBool = BeamBreak.get();
        SmartDashboard.putBoolean("BeamBreak", beamBreakBool);
        if (useBeamBreak && !beamBreakBool)
        {
            setIntakeStatus(IntakeStatus.STOPPED);
        }
    }
}