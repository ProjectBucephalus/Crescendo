package frc.lib.util;

import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RumbleController extends SubsystemBase{
    XboxController driver;
    XboxController coDriver;

    public enum Controllers {
        DRIVER,
        CODRIVER
    }

    public RumbleController(XboxController driver, XboxController coDriver) {
        this.driver = driver;
        this.coDriver = coDriver;
    }

    public void setRumble(Controllers con, double intensity, RumbleType type) {
        if (con == Controllers.DRIVER) {
            if (driver != null) {
                driver.setRumble(type, intensity);
            }
        }
        if (con == Controllers.CODRIVER) {
            if (coDriver != null) {
                coDriver.setRumble(type, intensity);
            }
        }
    }

    @Override
    public void periodic() {

    }
}
