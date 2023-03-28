package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
    private boolean debug = false;


    private TalonSRX intakeMotor = new TalonSRX(16);
    private double motorSpeed = 0;
    CTREConfigs configs = new CTREConfigs();

    private DigitalInput leftSensor;
    private DigitalInput rightSensor;

    private Debouncer leftDebouncer = new Debouncer(0.5);
    private Debouncer rightDebouncer = new Debouncer(0.5);
    // private static final Spark m_intakeRoller = new Spark(INTAKE_ROLLER_CAN_NUM

    public IntakeSubsystem() {
        intakeMotor.configAllSettings(configs.intakeMotorConfig);
        intakeMotor.setNeutralMode(NeutralMode.Brake);

        leftSensor = new DigitalInput(0);
        rightSensor = new DigitalInput(1);
    }


    public void setDebug(boolean debug) {
        this.debug = debug;
    }

    public void run(){
        motorSpeed = IntakeConstants.INTAKE_SPEED;
        IntakeConstants.intakeRunning = true;
        // System.out.println("RUN");
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
        SmartDashboard.putBoolean("cone in intake l", leftSensor.get());
        SmartDashboard.putBoolean("cone in intake r", rightSensor.get());
        intakeMotor.set(TalonSRXControlMode.PercentOutput, motorSpeed);
    }

    public boolean coneInIntake() {
        return leftDebouncer.calculate(leftSensor.get()) && rightDebouncer.calculate(rightSensor.get());
    }
}





