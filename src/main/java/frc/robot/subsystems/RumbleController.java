package frc.robot.subsystems;

import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RumbleController extends SubsystemBase{
    XboxController driver;
    XboxController coDriver;

    private boolean intakeRumble = false;
    private boolean aimRumble = false;

    public enum Controllers {
        DRIVER,
        CODRIVER
    }

    public enum RumbleStates 
    {
        INTAKE,
        AIM
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

            default:
                break;
        }
    }

    /**
     * Do not use outside of PointAndPathFindCommand or this subsystem
     * @param con Controller to set rumble of
     * @param intensity The intensity to set the rumble to
     */
    public void setRumble(Controllers con, double intensity) 
    {
        if (con == Controllers.DRIVER) {
            if (driver != null) {
                driver.setRumble(RumbleType.kBothRumble, intensity);
            }
        }
        if (con == Controllers.CODRIVER) {
            if (coDriver != null) {
                coDriver.setRumble(RumbleType.kBothRumble, intensity);
            }
        }
    }

    @Override
    public void periodic() {
        if (intakeRumble)
        {
            setRumble(Controllers.DRIVER, 1);
        }
        else if (aimRumble)
        {
            setRumble(Controllers.DRIVER, 0.5);
        }
        else 
        {
            setRumble(Controllers.DRIVER, 0);
        }
    }
}
