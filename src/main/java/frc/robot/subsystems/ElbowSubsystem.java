package frc.robot.subsystems;

public class ElbowSubsystem extends ServoMotorSubsystem {

    public ElbowSubsystem(ServoMotorSubsystemConfig config) {
        super(config);
    }

    @Override
    protected boolean allowPositiveMotion(double angle) {
        return true;
    }

    @Override
    protected boolean allowNegativeMotion(double angle) {
        return true;
    }
    
}
