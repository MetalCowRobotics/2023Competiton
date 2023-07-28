package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Patty {

    TalonSRX motor1 = new TalonSRX(11);

    public void runMotor(){
        motor1.set(TalonSRXControlMode.PercentOutput, 0.3);
    }

    public void stopMotor(){
        motor1.set(TalonSRXControlMode.PercentOutput, 0);
    }
    
}
