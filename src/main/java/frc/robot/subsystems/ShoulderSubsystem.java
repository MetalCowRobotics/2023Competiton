package frc.robot.subsystems;

public class ShoulderSubsystem extends ServoMotorSubsystem {

    public ShoulderSubsystem(ServoMotorSubsystemConfig config) {
        super(config);
    }

    @Override
    protected boolean allowPositiveMotion(double angle) {
        return false;
    }

    @Override
    protected boolean allowNegativeMotion(double angle) {
        return false;
    }
    
}
