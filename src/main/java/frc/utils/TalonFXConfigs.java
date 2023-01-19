package frc.utils;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class TalonFXConfigs {
    public static TalonFXConfiguration driveMotorTalonFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            true, // enabled/disabled
            35, // continuous current limit (amps)
            60, // peak current limit (amps)
            0.1 // peak current duration (seconds)
        );
      
        config.supplyCurrLimit = driveSupplyLimit;
        config.initializationStrategy = SensorInitializationStrategy.BootToZero;
        config.openloopRamp = 0.25; // open loop ramp time (seconds)
        config.closedloopRamp = 0.0; // closed loop ramp time (seconds)
        return config;
    }

    public static TalonFXConfiguration steerMotorTalonFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            true, // enabled/disabled
            25, // continuous current limit (amps)
            40, // peak current limit (amps)
            0.1 // peak current duration (seconds)
        );
      
        config.supplyCurrLimit = driveSupplyLimit;
        config.openloopRamp = 0.25;
        config.initializationStrategy = SensorInitializationStrategy.BootToZero;
        return config;
    }
}
