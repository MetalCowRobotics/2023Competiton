package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private boolean debug = false;

    private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);
    private double motorSpeed = 0;

    // private static final Spark m_intakeRoller = new Spark(INTAKE_ROLLER_CAN_NUM

    public IntakeSubsystem() {
    }

    public void setDebug(boolean debug) {
        this.debug = debug;
    }

    public void run(){
        motorSpeed = IntakeConstants.INTAKE_SPEED;
    }

    public void stop(){
        motorSpeed = 0;
    }

    @Override
    public void periodic() {
        intakeMotor.set(motorSpeed);
    }
}