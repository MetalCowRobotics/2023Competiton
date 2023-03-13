package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
    private boolean debug = false;


    private TalonSRX intakeMotor = new TalonSRX(16);
    private double motorSpeed = 0;
    CTREConfigs configs = new CTREConfigs();

    // private static final Spark m_intakeRoller = new Spark(INTAKE_ROLLER_CAN_NUM

    public IntakeSubsystem() {
        intakeMotor.configAllSettings(configs.intakeMotorConfig);
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
        intakeMotor.set(TalonSRXControlMode.PercentOutput, motorSpeed);
    }
}





