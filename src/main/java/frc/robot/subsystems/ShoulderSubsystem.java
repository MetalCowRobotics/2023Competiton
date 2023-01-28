package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ShoulderSubsystem extends SubsystemBase{
    AnalogPotentiometer pot = new AnalogPotentiometer(0, 180, 30);
    TalonSRX cim15 = new TalonSRX(15);

    public void startMotor(){
        cim15.set(TalonSRXControlMode.PercentOutput, .3);
    }

    public void startMotorReverse(){
        cim15.set(TalonSRXControlMode.PercentOutput, -0.3);        
    }
    
    public void stopMotor(){
        cim15.set(TalonSRXControlMode.PercentOutput, 0);
    }

}

