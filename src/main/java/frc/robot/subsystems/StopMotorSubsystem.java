package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

public class StopMotorSubsystem extends SubsystemBase{
    private TalonSRX m_coolMotor = new TalonSRX(15);


    public void stopMotor(){
        m_coolMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    @Override
    public void periodic() {
// This method will be called once per scheduler run
    }

        
    }
