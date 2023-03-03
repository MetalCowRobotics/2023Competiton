package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton moveToCenter = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton moveToLeft = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton moveToRight = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton autoLevel = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    /* Subsystems */
    private Swerve m_swerve;
    private ShoulderSubsystem m_shoulderSubsystem;
    // private final ElbowSubsystem m_elbowSubsystem;
    // private final WristSubsystem m_wristSubsystem;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_swerve = new Swerve();

        ServoMotorSubsystem.ServoMotorSubsystemConfig shoulderConfig = new ServoMotorSubsystem.ServoMotorSubsystemConfig();
        shoulderConfig.motorCanID = 15;
        shoulderConfig.stallCurentLimit = 30;
        shoulderConfig.freeCurentLimit = 30;
        shoulderConfig.kP = 0;
        shoulderConfig.kI = 0;
        shoulderConfig.kD = 0;
        shoulderConfig.positionTolerance = 2;
        shoulderConfig.minRPM = 0;
        shoulderConfig.maxRPM = 0;
        shoulderConfig.initialPosition = 0;
        shoulderConfig.reduction = 100;
        shoulderConfig.subsystemName = "Shoulder";

        m_shoulderSubsystem = new ShoulderSubsystem(shoulderConfig);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    public void configureButtonBindings() {
        /* Driver Buttons */
        m_swerve.setDefaultCommand(
            new TeleopSwerve(
                m_swerve, 
                () -> driver.getRawAxis(translationAxis), 
                () -> driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> moveToCenter.getAsBoolean(),
                () -> moveToLeft.getAsBoolean(),
                () -> moveToRight.getAsBoolean(),
                () -> autoLevel.getAsBoolean()
            )
        );
        zeroGyro.onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));
        m_swerve.enableVision();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new SequentialCommandGroup(new InstantCommand(() -> m_swerve.zeroGyro()), new DisableVision(m_swerve), new DriveToPoint(m_swerve, -2.6, 0, 0), new EnableVision(m_swerve));
    }
}
