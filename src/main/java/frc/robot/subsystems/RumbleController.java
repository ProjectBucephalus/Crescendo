package frc.robot.subsystems;

import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RumbleController extends SubsystemBase
{
    XboxController driver;
    XboxController coDriver;

    private boolean intakeRumble = false;
    private boolean aimRumble = false;
    private boolean shootRumble = false;

    public enum Controllers 
    {
        DRIVER,
        CODRIVER
    }

    public enum RumbleStates 
    {
        INTAKE,
        AIM,
        SHOOTREADY
    }

    public RumbleController(XboxController driver, XboxController coDriver) 
    {
        this.driver = driver;
        this.coDriver = coDriver;
    }

    public void setRumbleStatus(RumbleStates rumbleToSet, Boolean rumbleValue)
    {
        switch (rumbleToSet) 
        {
            case INTAKE:
                intakeRumble = rumbleValue;
                break;
            
            case AIM:
                aimRumble = rumbleValue;
                break;

            case SHOOTREADY:
                shootRumble = rumbleValue;
                break;

            default:
                break;
        }
    }

    /**
     * Do not use outside of this subsystem
     * @param con Controller to set rumble of
     * @param intensity The intensity to set the rumble to
     */
    private void setRumble(Controllers con, double intensity, RumbleType rumbleType) 
    {
        if (con == Controllers.DRIVER) 
        {
            if (driver != null) 
            {
                driver.setRumble(rumbleType, intensity);
            }
        }
        if (con == Controllers.CODRIVER) 
        {
            if (coDriver != null) 
            {
                coDriver.setRumble(rumbleType, intensity);
            }
        }
    }

    private void driverRumbleControl()
    {
        if (intakeRumble)
        {
            setRumble(Controllers.DRIVER, 1, RumbleType.kRightRumble);
        }
        else 
        {
            setRumble(Controllers.DRIVER, 0, RumbleType.kRightRumble);
        }
        if (aimRumble)
        {
            setRumble(Controllers.DRIVER, 1, RumbleType.kLeftRumble);
        }
        else
        {
            setRumble(Controllers.DRIVER, 0, RumbleType.kLeftRumble);
        }
    }
    
    private void coDriverRumbleControl()
    {
        if (shootRumble)
        {
            setRumble(Controllers.CODRIVER, 1, RumbleType.kBothRumble);
        }
        else 
        {
            setRumble(Controllers.CODRIVER, 0, RumbleType.kBothRumble);
        }
    }

    @Override
    public void periodic() 
    {
        driverRumbleControl();
        coDriverRumbleControl();
    }
}
