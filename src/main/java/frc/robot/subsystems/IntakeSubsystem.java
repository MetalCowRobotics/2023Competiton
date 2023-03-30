package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
    private boolean debug = false;


    private TalonSRX intakeMotor = new TalonSRX(16);
    private double motorSpeed = 0;
    int i=0;
    CTREConfigs configs = new CTREConfigs();

    private DigitalInput coneSensor;
    private DigitalInput cubeSensor;

    private Debouncer coneDebouncer = new Debouncer(0.5);
    private Debouncer cubeDebouncer = new Debouncer(0.5);

    public IntakeSubsystem() {
        intakeMotor.configAllSettings(configs.intakeMotorConfig);
        intakeMotor.setNeutralMode(NeutralMode.Brake);

        coneSensor = new DigitalInput(IntakeConstants.CONE_SENSOR_DIO);
        cubeSensor = new DigitalInput(IntakeConstants.CUBE_SENSOR_DIO);
    }


    public void setDebug(boolean debug) {
        this.debug = debug;
    }

    // cone intake, cube eject
    public void run(){
        motorSpeed = IntakeConstants.INTAKE_SPEED;
        IntakeConstants.intakeRunning = true;
    }

    // cone eject, cube intake
    public void runReverse() {
        motorSpeed = -IntakeConstants.INTAKE_SPEED;
        IntakeConstants.intakeRunning = true;
    }


    public void stop(){
        motorSpeed = 0;
        IntakeConstants.intakeRunning = false;
    }

    public boolean coneInIntake() {
        return !coneDebouncer.calculate(coneSensor.get());
    }

    public boolean cubeInIntake() {
        return !cubeDebouncer.calculate(cubeSensor.get());
    }

    public void eject() {
        if (cubeInIntake()) {
            run();
        } else {
            runReverse();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("cone in intake", coneInIntake());
        SmartDashboard.putBoolean("cube in intake", cubeInIntake());
        SmartDashboard.putBoolean("cone sensor", coneSensor.get());
        SmartDashboard.putBoolean("cube sensor", cubeSensor.get());
        intakeMotor.set(TalonSRXControlMode.PercentOutput, motorSpeed);
    }
}





