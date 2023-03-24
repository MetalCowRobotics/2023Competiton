package frc.robot.subsystems;

public class ShoulderSubsystem extends ServoMotorSubsystem {

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
    
}
