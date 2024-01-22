package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

public class Climber {
    private static CANSparkMax m_Climber = new CANSparkMax(Constants.Climber.climbMotorID, CANSparkLowLevel.MotorType.kBrushed);

    private static double motorSpeed = Constants.Climber.climbMotorSpeed;

    //climber go up
    public static void climberUp() 
    {
        m_Climber.set(motorSpeed);
    }
    
    //climber go down
    public static void climberDown() 
    {
        m_Climber.set(-motorSpeed);
    }

    //climber stop
    public static void climberStop() 
    {
        m_Climber.stopMotor();
    }

}
