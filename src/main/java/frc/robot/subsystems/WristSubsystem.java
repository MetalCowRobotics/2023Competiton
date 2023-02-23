package frc.robot.subsystems;

public class WristSubsystem extends ServoMotorSubsystem {

    public WristSubsystem(ServoMotorSubsystemConfig config) {
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
