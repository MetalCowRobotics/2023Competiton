package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElbowConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

public class ElbowSubsystem extends SubsystemBase{
    TalonSRX cim15 = new TalonSRX(15);
    private double motorSpeed = 0;
    DigitalInput elbowlimitSwitch = new DigitalInput(ElbowConstants.elbowSensorID);
    public void run(){
        motorSpeed = 0.3;
    }

    public void runReverse() {
        motorSpeed = -0.3;
    }

    public void stop(){
        motorSpeed = 0;
    }

    public boolean isReached(){
        return elbowlimitSwitch.get();
    }
    
    @Override
    public void periodic() {
        cim15.set(TalonSRXControlMode.PercentOutput, motorSpeed);
        System.out.println(isReached());
    }
    
}
