package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.RumbleController;
import frc.lib.util.RumbleController.Controllers;
import frc.robot.Constants;
import frc.lib.math.Conversions;
import frc.robot.CTREConfigs;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * intake subsystem 
 * @author 5985
 */
public class Intake extends SubsystemBase 
{
    // Declarations of all the motor controllers
    public TalonFX mIntake = new TalonFX(IDConstants.Intooter.Intake.mIntakeID);
    public TalonFX mIndexer = new TalonFX(IDConstants.Intooter.Intake.mIndexerID);
    public VictorSPX mStabilser = new VictorSPX(IDConstants.Climber.mStabiliserID);

    // Declaration of the beam break digital input
    public DigitalInput BeamBreak = new DigitalInput(IDConstants.Intooter.Intake.beamBreakID);
    public DigitalInput StabilserLimit = new DigitalInput(IDConstants.Intooter.Intake.stabiliserLimitID);



    // Booleans regarding the beam braek
    private boolean beamBreakBool = false;
    private boolean useBeamBreak = false;

    // For rumbling with note flag
    private boolean hasRumbled = false;

    private boolean useStabiliserLimitSwitch = true;

    private boolean doRumbleWithNote = false;
    private boolean prevBeamBrakeState = false;

    private boolean timerHasReset = false;


    private RumbleController s_RumbleController;

    Timer m_timer = new Timer();
    

    /** 
     * Enum representing the roller status of the Intake 
     * (Spinning inwards, spinning outwards, spinning inwards with beam break control, stopped, or spinning inwards to feed for shooting)
     * @author 5985
     */
    public enum IntakePosition {
        STOWED,
        DEPLOYED,
        AMP,
        TRAP,
        SPEAKER
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

    public Intake(RumbleController s_RumbleController) 
    {
        this.s_RumbleController = s_RumbleController;
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
        //System.out.println("setIntakeStatus Getting set");
        
        switch (status) 
        {
            case IN_FOR_SHOOTING:
                setIndexerState(IndexerState.IN_FOR_SHOOTING);
                setIntakeSpeed(Constants.Intake.intakeSpeedShoot, false);
                useBeamBreak = false;
                break;
            case IN:
                setIndexerState(IndexerState.IN);
                setIntakeSpeed(Constants.Intake.intakeSpeedIn, false);
                useBeamBreak = false;
                break;
            case OUT:
                setIndexerState(IndexerState.OUT);
                setIntakeSpeed(Constants.Intake.intakeSpeedOut, false);
                useBeamBreak = false;
                break;
            case IN_WITH_BEAM_BREAK:
                
                setIndexerState(IndexerState.IN_WITH_BEAM_BREAK);
                setIntakeSpeed(Constants.Intake.intakeSpeedInWithLimit, true);
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
                mIndexer.set(Constants.Intake.indexSpeedIn);
                break;
        
            case OUT:
                mIndexer.set(Constants.Intake.indexSpeedOut);
                
                break;
            case STOPPED:
                mIndexer.set(0.0);
                break;
            case IN_WITH_BEAM_BREAK:
                mIndexer.set(Constants.Intake.indexSpeedInWithLimit);
                break;
            case IN_FOR_SHOOTING:
                mIndexer.set(Constants.Intake.indexSpeedShoot);
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
                mStabilser.set(ControlMode.PercentOutput, 1);
                useStabiliserLimitSwitch = true;
                break;
            case OUT:
                mStabilser.set(ControlMode.PercentOutput, -1);
                useStabiliserLimitSwitch = false;
                break;
            case STOPPED:
                mStabilser.set(ControlMode.PercentOutput, 0);
                useStabiliserLimitSwitch = false;
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

    public void rumbleWithNote(Boolean doRumbleWithNote) {
        this.doRumbleWithNote = doRumbleWithNote;
    }
    
    @Override
    public void periodic() 
    {
        // Sets beamBreakBool to the value of the Beam Break
        beamBreakBool = BeamBreak.get();
        SmartDashboard.putBoolean("Stabiliser Limit", StabilserLimit.get());
        SmartDashboard.putNumber("Indexer RPS", mIndexer.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake RPS", mIntake.getVelocity().getValueAsDouble());
        

        if (doRumbleWithNote && !getBeamBreak() && !hasRumbled) {
            // start the rumble with intensity 1
            s_RumbleController.setRumble(Controllers.DRIVER, 1, RumbleType.kBothRumble);
            //System.out.println("Rumble started.");
            hasRumbled = true; // set the flag to true
        } else if (getBeamBreak() || hasRumbled) {
            // stop the rumble
            s_RumbleController.setRumble(Controllers.DRIVER, 0, RumbleType.kBothRumble);
            //System.out.println("Rumble stopped.");
            hasRumbled = false; // reset the flag to false
        }
        
        // Prints the beamBreakBool to the Smart Dashboard
        SmartDashboard.putBoolean("BeamBreak", beamBreakBool);
        
        // Stops the Intake rollers if the beam break is tripped and it is set to be using the beam break for control
        
        if (useBeamBreak && !beamBreakBool)
        {
            if (timerHasReset == false) {
                timerHasReset = true;
            m_timer.restart();
            }
            
            if (m_timer.hasElapsed(Constants.Intake.extraIntakeTime)) {
                setIntakeStatus(IntakeStatus.STOPPED);
                timerHasReset = false;
            }
            
        }
        if (useStabiliserLimitSwitch)
        {
            if (!StabilserLimit.get()) {
                setStabliserPos(StabiliserPos.STOPPED);
            }
        }
    }
}