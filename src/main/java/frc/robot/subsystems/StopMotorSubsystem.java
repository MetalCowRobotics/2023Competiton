package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class StopMotorSubsystem extends SubsystemBase{
    private CANSparkMax m_coolMotor = new CANSparkMax(13, MotorType.kBrushless);


    public void stopMotor() {
        m_coolMotor.set(0.0);
    }

    @Override
    public void periodic() {
// This method will be called once per scheduler run
    }

        
    }
