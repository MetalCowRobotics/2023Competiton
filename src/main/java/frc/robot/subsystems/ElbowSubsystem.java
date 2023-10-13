package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class ElbowSubsystem extends ServoMotorSubsystem {

    AnalogPotentiometer pot;

    public ElbowSubsystem(ServoMotorSubsystemConfig config) {
        super(config);
        // pot = new AnalogPotentiometer(Constants.ArmConstants.Offsets.ELBOW_POT_ANALOG_ID, 3600, -Constants.ArmConstants.Offsets.ELBOW_POT_OFFSET);
    }

    @Override
    protected boolean allowPositiveMotion(double angle) {
        return true;
    }

    @Override
    protected boolean allowNegativeMotion(double angle) {
        return true;
    }

    // @Override
    // public double getCurrentAngle() {
    //     return -pot.get();
    // }

    // @Override
    // public void periodic() {
        
    // }
    
}
