package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    DigitalInput coneSensor = new DigitalInput(IntakeConstants.coneSensorID);
    DigitalInput cubeSensor = new DigitalInput(IntakeConstants.cubeSensorID);
    
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
        IntakeConstants.intakeRunning = true;
    }

    public void runReverse() {
        motorSpeed = -(IntakeConstants.INTAKE_SPEED);
        IntakeConstants.intakeRunning = true;
    }

    public void stop(){
        motorSpeed = 0;
        IntakeConstants.intakeRunning = false;
    }

    @Override
    public void periodic() {
        intakeMotor.set(motorSpeed);
    }

    public boolean isConePresent(){
       return coneSensor.get();
    }
    public boolean isCubePresent(){
        return cubeSensor.get();
     }
}
