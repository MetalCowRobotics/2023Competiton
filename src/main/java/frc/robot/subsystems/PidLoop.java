package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;




public class PidLoop {
    private CANSparkMax m_motor = new CANSparkMax(15, MotorType.kBrushless);
    private Type kQuadrature;
    private RelativeEncoder encoder =  m_motor.getEncoder(kQuadrature, 42);
    private SparkMaxPIDController pid;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private double goalSpeed;
    public void PIDLOOP() {
        pid = m_motor.getPIDController();
        encoder = m_motor.getEncoder();

        SmartDashboard.putNumber("Speed Correction", 0);

        // PID coefficients
        kP = 0.00012;
        kI = 0.0000004;
        kD = 0.0002;
        kIz = 2000;
        kFF = 0.000015;
        kMaxOutput = 0.5;
        kMinOutput = -0.5;

        // set PID coefficients
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
        pid.setIZone(kIz);
        pid.setFF(kFF);
        pid.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput); 
        }
}
