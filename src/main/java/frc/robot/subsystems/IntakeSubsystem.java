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
    private AnalogPotentiometer cubeSensor;
    private Ultrasonic cubeUltrasonic;
    private PWM cubeUltraPwm;

    private Debouncer coneDebouncer = new Debouncer(0.5);

    public IntakeSubsystem() {
        intakeMotor.configAllSettings(configs.intakeMotorConfig);
        intakeMotor.setNeutralMode(NeutralMode.Brake);

        coneSensor = new DigitalInput(IntakeConstants.CONE_SENSOR_DIO);
        cubeSensor = new AnalogPotentiometer(IntakeConstants.CUBE_SENSOR_ANALOG, 4096, 0);
        cubeUltraPwm = new PWM(2);
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
        return cubeSensor.get() < IntakeConstants.CUBE_INSIDE_THRESHOLD;
    }

    public void eject() {
        if (coneInIntake()) {
            runReverse();
        } else {
            run();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("cone in intake", coneInIntake());
        SmartDashboard.putBoolean("cube in intake", cubeInIntake());
        SmartDashboard.putBoolean("cone sensor", coneSensor.get());
        SmartDashboard.putNumber("cube sensor", cubeUltraPwm.getRaw());
        SmartDashboard.putNumber("intake count", i);
        i++;
        intakeMotor.set(TalonSRXControlMode.PercentOutput, motorSpeed);
    }
}





