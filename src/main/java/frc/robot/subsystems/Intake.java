package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake 
{
    private static CANSparkMax m_IntakeAngle = new CANSparkMax(9, CANSparkLowLevel.MotorType.kBrushed);
    private static CANSparkMax m_intake = new CANSparkMax(8, CANSparkLowLevel.MotorType.kBrushed);
    static DigitalInput topLimitSwitch = new DigitalInput(1);
    static DigitalInput bottomLimitSwitch = new DigitalInput(2);
    //replace device/channel id when we know what it is
    //angle has 2 motors so set up follow for one of  them    

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
       if (topLimitSwitch.get()) 
       {
            m_IntakeAngle.set(0);
       } else {
            m_IntakeAngle.set(0.20);
       }
    }
// when function is triggered it will check if the limit switch is pressed
    public static void setIntakeAngleStop()
    {m_IntakeAngle.set(0);}

    public static void IntakeIn()
    {
        m_intake.set(1);
    }

    public static void IntakeStop()
    {
        m_intake.set(0);
    }
    public static void intakeOut()
    {
        m_intake.set(-1);
    }
    //sets the speed for the intake

    public static void setIntakeStowed()
    {
        m_IntakeAngle.set(1);
    }

    
}
