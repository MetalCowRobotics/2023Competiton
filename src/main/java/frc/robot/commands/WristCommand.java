package frc.robot.commands;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristCommand extends SubsystemBase{
    CANSparkMax motor = new CANSparkMax(17, MotorType.kBrushless);
    public void positive () {
        motor.set(1);
     }    
     public void negative () {
        motor.set(-1);
     }    
    }