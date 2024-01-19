package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake 
{
    private static CANSparkMax m_IntakeAngle = new CANSparkMax(9, CANSparkLowLevel.MotorType.kBrushed);
    static DigitalInput topLimitSwitch = new DigitalInput(1);
    static DigitalInput bottomLimitSwitch = new DigitalInput(2);
    //replace device/channel id when we know what it is
    


    public static void setIntakeDown()
    {
        if (bottomLimitSwitch.get())
        {
            m_IntakeAngle.set(0);
        } else {
            m_IntakeAngle.set(-0.20);
        }
    }

    public static void setIntakeUp()
    {
        



    }


    //{m_IntakeAngle.set(0.2);}
}
