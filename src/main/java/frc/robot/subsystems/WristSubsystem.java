package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.KalmanFilter;

public class WristSubsystem extends ServoMotorSubsystem {

    AnalogPotentiometer pot;
    KalmanFilter kf = new KalmanFilter(0.1, 1.0);

    public WristSubsystem(ServoMotorSubsystemConfig config) {
        super(config);
        pot = new AnalogPotentiometer(Constants.ArmConstants.Offsets.WRIST_POT_ANALOG_ID, 3600, -Constants.ArmConstants.Offsets.WRIST_POT_OFFSET);
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

    @Override
    public double getCurrentAngle() {
        return kf.lastMeasurement();
    }

    @Override
    public void periodic() {
        super.periodic();
        double adjustedAngle = kf.filter(pot.get(), super.getCurrentAngle());
        SmartDashboard.putNumber("wrist pot reading", pot.get());
        SmartDashboard.putNumber("adjusted wrist reading", adjustedAngle);
        SmartDashboard.putNumber("adjusted wrist reading last", kf.lastMeasurement());
    }
}
