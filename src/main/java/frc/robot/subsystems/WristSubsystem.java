package frc.robot.subsystems;

public class WristSubsystem extends ServoMotorSubsystem {

    private double wristAngle = 0;

    public WristSubsystem(ServoMotorSubsystemConfig config) {
        super(config);
    }

    public void cubeFloorIntakePosistion() {
        // targetAngle
    }

    @Override
    protected boolean allowPositiveMotion(double angle) {
        return angle >= -10 && angle <= 135;
        // return true;
    }

    @Override
    protected boolean allowNegativeMotion(double angle) {
        return angle >= -10 && angle <= 135;
        // return true;
    }
    
    public void wristUp() {
        super.setTarget(getTargetAngle() + 2.0);
    }

    public void wristDown() {
        super.setTarget(getTargetAngle() - 2.0);
    }
}
