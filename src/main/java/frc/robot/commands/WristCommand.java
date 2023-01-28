package frc.robot.commands;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase{
    CANSparkMax motor = new CANSparkMax(17, MotorType.kBrushless);
    public void positive () {
        motor.set(1);
     }    
     public void negative () {
        motor.set(-1);
     }    
    }