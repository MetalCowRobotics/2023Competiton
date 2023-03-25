package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;

public class ShoulderSubsystem extends ServoMotorSubsystem {
    DigitalInput irSensor = new DigitalInput(0);
    DigitalInput irSensor2 = new DigitalInput(1);

    public ShoulderSubsystem(ServoMotorSubsystemConfig config) {
        super(config);
    }

    @Override
    protected boolean allowPositiveMotion(double angle) {
        return angle >= 0;
    }

    @Override
    protected boolean allowNegativeMotion(double angle) {
        return angle <= 180;
    }

    @Override
    public void periodic(){
        System.out.println( irSensor.get());
    }
    
}
