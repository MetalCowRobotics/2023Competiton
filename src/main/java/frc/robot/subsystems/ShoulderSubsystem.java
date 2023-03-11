package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ShoulderConstants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ShoulderSubsystem extends SubsystemBase{
    private double motorSpeed = 0;
    DigitalInput shoulderLimitSwitch = new DigitalInput(ShoulderConstants.shoulderSensorID);
    TalonSRX cim15 = new TalonSRX(15);

    public void startMotor(){
        motorSpeed = 0.3;
        // cim15.set(TalonSRXControlMode.PercentOutput, .3);
    }

    public void startMotorReverse(){
        motorSpeed = -0.3;
        // cim15.set(TalonSRXControlMode.PercentOutput, -.3);        
    }
    
    public void stopMotor(){
        motorSpeed = 0;
        // cim15.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public boolean isReached(){
        return shoulderLimitSwitch.get();
    }
    
    @Override
    public void periodic() {
        cim15.set(TalonSRXControlMode.PercentOutput, motorSpeed);
        System.out.println(isReached());
    }
}

