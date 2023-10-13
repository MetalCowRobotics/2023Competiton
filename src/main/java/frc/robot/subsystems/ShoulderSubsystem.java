package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class ShoulderSubsystem extends ServoMotorSubsystem {

    AnalogPotentiometer pot;

    public ShoulderSubsystem(ServoMotorSubsystemConfig config) {
        super(config);
        // pot = new AnalogPotentiometer(Constants.ArmConstants.Offsets.SHOULDER_POT_ANALOG_ID, 3600, -Constants.ArmConstants.Offsets.SHOULDER_POT_OFFSET);
    }

    @Override
    protected boolean allowPositiveMotion(double angle) {
        return angle >= 0;
    }

    @Override
    protected boolean allowNegativeMotion(double angle) {
        return angle <= 180;
    }

    // @Override
    // public double getCurrentAngle() {
    //     return -pot.get();
    // }
    
}
