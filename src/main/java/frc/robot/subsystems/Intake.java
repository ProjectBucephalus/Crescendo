package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IDConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Intake subsystem, handling the intake rollers, indexer rollers, and stabiliser bar (latter should probably be in Climber.java)
 * @author 5985
 */
public class Intake extends SubsystemBase 
{
    // Declarations of all the motor controllers
    public TalonFX mIntake = new TalonFX(IDConstants.Intooter.Intake.mIntakeID);
    public TalonFX mIndexer = new TalonFX(IDConstants.Intooter.Intake.mIndexerID);
    public VictorSPX mStabilser = new VictorSPX(IDConstants.Climber.mStabiliserID);

    // Declaration of the beam break digital input
    public DigitalInput BeamBreak = new DigitalInput(9);


    // Booleans regarding the beam braek
    private boolean beamBreakBool = false;
    private boolean useBeamBreak = false;

    /** 
     * Enum representing the roller status of the Intake 
     * (Spinning inwards, spinning outwards, spinning inwards with beam break control, stopped, or spinning inwards to feed for shooting)
     * @author 5985
     */
    public enum IntakeStatus 
    {
        IN,
        OUT,
        IN_WITH_BEAM_BREAK,
        STOPPED,
        IN_FOR_SHOOTING
    };

    /** 
     * Enum representing the status of the Stabiliser 
     * (Moving outwards, moving inwards, or not moving)
     * @author 5985
     */
    public enum StabiliserPos 
    {
        OUT, 
        IN,
        STOPPED
    };

    /** 
     * Enum representing the status of the Indexer
     * (Stopped, spinning inwards, spinning outwards, spinning inwards with beam break control, or spinning inwards to feed for shooting)
     * @author 5985
     */
    public enum IndexerState 
    {
        STOPPED,
        IN,
        OUT,
        IN_WITH_BEAM_BREAK,
        IN_FOR_SHOOTING

    };

    public Intake() 
    {
        // Displays values in Smart Dashboard
        SmartDashboard.putNumber("topShooterSpeed", 1);
        SmartDashboard.putNumber("bottomShooterSpeed", 1);
        
        // SmartDashboard.putNumber("pivotPosition", 1.2);
    }

    /**
     * Sets the speed of the Intake motor
     * 
     * @param speed Intake motor speed [-1..1]
     * @param useBeamBreak (Unused) Set true to stop intake when note is detected
     */
    public void setIntakeSpeed(double speed, boolean useBeamBreak) 
    {
        mIntake.set(speed);
    }

    /**
     * Sets the speed of the Intake motor based on an enum 
     * 
     * @param status Enum corresponding to intake motor speed and related values
     */
    public void setIntakeStatus(IntakeStatus status) 
    {
        SmartDashboard.putString("Intake Status", status.name());
        System.out.println("setIntakeStatus Getting set");
        
        switch (status) 
        {
            case IN_FOR_SHOOTING:
                setIndexerState(IndexerState.IN_FOR_SHOOTING);
                setIntakeSpeed(1, false);
                useBeamBreak = false;
                break;
            case IN:
                setIndexerState(IndexerState.IN);
                setIntakeSpeed(0.50, false);
                useBeamBreak = false;
                break;
            case OUT:
                setIndexerState(IndexerState.OUT);
                setIntakeSpeed(-0.35, false);
                useBeamBreak = false;
                break;
            case IN_WITH_BEAM_BREAK:
                
                setIndexerState(IndexerState.IN_WITH_BEAM_BREAK);
                setIntakeSpeed(0.75, true);
                useBeamBreak = true;
                break;
            case STOPPED:
                setIndexerState(IndexerState.STOPPED);
                setIntakeSpeed(0, false);
                useBeamBreak = false;
                break;
            default:
                break;
        }
    }

    /**
     * sets the status of the indexer motor 
     * TODO replace values with constants
     * @param pos Enum value corresponding to indexer speeds
     * @author 5985
     * @author Aidan
     */
    public void setIndexerState(IndexerState pos) 
    {
        SmartDashboard.putString("indexer Status", pos.name());
        switch (pos) 
        {
            // this means that we are running the intake in. We need to do logic on what we actually need to do. 
            case IN:
                mIndexer.set(0.25);
                break;
        
            case OUT:
                mIndexer.set(-0.35);
                
                break;
            case STOPPED:
                mIndexer.set(0.0);
                break;
            case IN_WITH_BEAM_BREAK:
                mIndexer.set(-0.4);
                break;
            case IN_FOR_SHOOTING:
                mIndexer.set(1);
                break;
        }
    }

    /**
     * Sets the status of the Stabiliser
     * (Moving inwards, moving outwards, or not moving)
     * @param pos Enum value corresponding to Stabiliser status
     * @author 5985
     */
    public void setStabliserPos(StabiliserPos pos) 
    {
        SmartDashboard.putString("Stabliser Status", pos.name());
        switch (pos) 
        {
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

    /** 
     * Gets the value of the Beam Break
     * @author 5985
     */
    public boolean getBeamBreak() 
    {
        return beamBreakBool;
    }
    
    @Override
    public void periodic() 
    {
        // Sets beamBreakBool to the value of the Beam Break
        beamBreakBool = BeamBreak.get();
        
        // Prints the beamBreakBool to the Smart Dashboard
        SmartDashboard.putBoolean("BeamBreak", beamBreakBool);
        
        // Stops the Intake rollers if the beam break is tripped and it is set to be using the beam break for control
        if (useBeamBreak && !beamBreakBool)
        {
            setIntakeStatus(IntakeStatus.STOPPED);
        }
    }
}