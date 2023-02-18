package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



public class ShoulderSubsystem extends SubsystemBase{
    AnalogPotentiometer pot = new AnalogPotentiometer(0, 180, 30);
    CANSparkMax neo = new CANSparkMax(15, MotorType.kBrushless);
    DigitalInput input = new DigitalInput(0);
    // Limit switch on DIO 2
    DigitalInput limit = new DigitalInput(2);
    DigitalInput limit2 = new DigitalInput(3);

    @Override
    public void periodic() {
        if(!limit.get() && neo.get()>0 || (!limit.get() && neo.get()<0)){
            neo.set(0);
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

