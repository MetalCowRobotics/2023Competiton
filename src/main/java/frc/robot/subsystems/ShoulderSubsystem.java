package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



public class ShoulderSubsystem extends SubsystemBase{
    CANSparkMax neo = new CANSparkMax(10, MotorType.kBrushless);
    DigitalInput limit = new DigitalInput(2);
    DigitalInput limit2 = new DigitalInput(3);


    @Override
    public void periodic() {
        neo.set(0.30);
    
        if(limit.get()){
            neo.set(0);
            System.out.println("Positive");
        }
        else if (limit2.get()){
            neo.set(0);
            System.out.println("Negative");
        }
    }

    public void startMotor(){
        neo.set(.3);
    }

    public void startMotorReverse(){
        neo.set(-.3);        
    }
    
    public void stopMotor(){
        neo.set(0);
    }
}

