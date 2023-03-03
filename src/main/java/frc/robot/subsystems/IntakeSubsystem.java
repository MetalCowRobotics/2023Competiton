package frc.robot.subsystems;

import javax.print.attribute.HashPrintJobAttributeSet;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private boolean debug = false;

    private TalonSRX intakeMotor = new TalonSRX(15);
    private double motorSpeed = 0; 
    private DigitalInput cube = new DigitalInput(0);
    private DigitalInput cone = new DigitalInput(1);
    
    

    // private static final Spark m_intakeRoller = new Spark(INTAKE_ROLLER_CAN_NUM

    public IntakeSubsystem() {
    }

    public void setDebug(boolean debug) {
        this.debug = debug;
    }

    public void run(){
        if ((cube.get()== true) || (cone.get()== true)){
            motorSpeed = .1;
        }else{
            motorSpeed = IntakeConstants.INTAKE_SPEED;
            IntakeConstants.intakeRunning = true;
        }
       

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
        intakeMotor.set(TalonSRXControlMode.PercentOutput, motorSpeed); // runs the motor at 50% power
    }
}
