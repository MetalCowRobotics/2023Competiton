package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class WristSubsystem extends SubsystemBase{
    CANSparkMax motor = new CANSparkMax(17, MotorType.kBrushless);
    public void positive () {
        motor.set(1);
     }    
     public void negative () {
        motor.set(-1);
     }    
    }