package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

public class StartMotorSubsystem extends SubsystemBase{
    private TalonSRX m_coolMotor = new TalonSRX(15);


    public void startMotor(){
        m_coolMotor.set(TalonSRXControlMode.PercentOutput, 0.3);
    }

    @Override
    public void periodic() {
// This method will be called once per scheduler run
    }

        
    }
