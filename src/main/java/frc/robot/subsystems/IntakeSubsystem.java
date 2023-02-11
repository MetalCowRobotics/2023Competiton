package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    TalonSRX m_intakeMotor = new TalonSRX(15);

    public void startMotor(){
        m_intakeMotor.set(TalonSRXControlMode.PercentOutput, .5);
    }

    public void startMotorReverse(){
        m_intakeMotor.set(TalonSRXControlMode.PercentOutput, -.5);
    }

    public void stopMotor(){
        m_intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
    @Override
    public void periodic(){

    }

    
}
