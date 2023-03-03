package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DisableVision;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.EnableVision;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ServoMotorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristSubsystem;

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
    
    /* Operator Buttons */
    private final JoystickButton cubeSubstationIntakePosition = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    // private final JoystickButton coneSubstationIntakePosition = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton cubeFloorIntakePosition = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton coneFloorIntakePosition = new JoystickButton(operator, XboxController.Button.kX.value);
    // private final JoystickButton lowScoringPosition = new JoystickButton(operator, XboxController.Button.kB.value);
    // private final JoystickButton midScoringPosition = new JoystickButton(operator, XboxController.Button.kY.value);
    // private final JoystickButton stow = new JoystickButton(operator, XboxController.Button.kA.value);
    // private final JoystickButton stopstow = new JoystickButton(operator, XboxController.Button.kB.value);

    /* Subsystems */
    private Swerve m_swerve = new Swerve();
    private ShoulderSubsystem m_shoulderSubsystem;
    private ElbowSubsystem m_elbowSubsystem;
    private WristSubsystem m_wristSubsystem;
    private IntakeSubsystem m_IntakeSubsystem;

    /* Autos */
    private SendableChooser<Command> m_autoSelector;               
    
    private Command chargeStationScoreMobilityDock = new SequentialCommandGroup(
        new InstantCommand(() -> m_swerve.zeroGyro()),
        new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
        new DisableVision(m_swerve), 
        new DriveToPoint(m_swerve, -4.5, 0, 0),
        new DriveToPoint(m_swerve, -2.6, 0, 0),
        new EnableVision(m_swerve)
    );
    private Command chargeStationScoreDock = new SequentialCommandGroup(
        new InstantCommand(() -> m_swerve.zeroGyro()),
        new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
        new DisableVision(m_swerve), 
        new DriveToPoint(m_swerve, -2.6, 0, 0),
        new EnableVision(m_swerve)
    );
    private Command chargeStationDock = new SequentialCommandGroup(
        new InstantCommand(() -> m_swerve.zeroGyro()),
        new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
        new DisableVision(m_swerve), 
        new DriveToPoint(m_swerve, -2.6, 0, 0),
        new EnableVision(m_swerve)
    );

    private Command substationScoreMobilityDockBlue = new SequentialCommandGroup(
        new InstantCommand(() -> m_swerve.zeroGyro()),
        new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
        new DisableVision(m_swerve), 
        new DriveToPoint(m_swerve, -4.3, 0, 0),
        new DriveToPoint(m_swerve, -4.3, 2, 0),
        new DriveToPoint(m_swerve, -2.6, 2, 0),
        new EnableVision(m_swerve)
    );

    private Command substationScoreMobilityBlue = new SequentialCommandGroup(
        new InstantCommand(() -> m_swerve.zeroGyro()),
        new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
        new DisableVision(m_swerve), 
        new DriveToPoint(m_swerve, -4.3, 0, 0),
        new EnableVision(m_swerve)
    );

    private Command cableRunScoreMobilityBlue = new SequentialCommandGroup(
        new InstantCommand(() -> m_swerve.zeroGyro()),
        new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
        new DisableVision(m_swerve), 
        new DriveToPoint(m_swerve, -4.3, 0, 0),
        new EnableVision(m_swerve)
    );

    private Command noAuto = new InstantCommand(() -> m_swerve.zeroGyro());


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_swerve = new Swerve();
        m_IntakeSubsystem = new IntakeSubsystem();
        m_autoSelector = new SendableChooser<Command>();
        m_autoSelector.addOption("Charge Station Score + Dock", chargeStationScoreDock);
        m_autoSelector.addOption("Charge Station Score + Mobility + Dock", chargeStationScoreMobilityDock);
        m_autoSelector.addOption("Charge Station Dock", chargeStationDock);
        m_autoSelector.addOption("Blue Substation Score + Mobility + Dock", substationScoreMobilityDockBlue);
        m_autoSelector.addOption("Blue Substation Score + Mobility", substationScoreMobilityBlue);
        m_autoSelector.addOption("Cable Run Score Mobility", cableRunScoreMobilityBlue);
        m_autoSelector.setDefaultOption("None", noAuto);

        ServoMotorSubsystem.ServoMotorSubsystemConfig shoulderConfig = new ServoMotorSubsystem.ServoMotorSubsystemConfig();
            shoulderConfig.motorCanID = 15;
            shoulderConfig.stallCurentLimit = 45;
            shoulderConfig.freeCurentLimit = 45;
            shoulderConfig.kP = 0.0002;
            shoulderConfig.kI = 0;
            shoulderConfig.kD = 0;
            shoulderConfig.positionTolerance = 2;
            shoulderConfig.minRPM = 1300;
            shoulderConfig.maxRPM = 1500;
            shoulderConfig.initialPosition = 0;
            shoulderConfig.reduction = 100 * (46.0 / 12.0);
            shoulderConfig.subsystemName = "Shoulder";

        ServoMotorSubsystem.ServoMotorSubsystemConfig elbowConfig = new ServoMotorSubsystem.ServoMotorSubsystemConfig();
            elbowConfig.motorCanID = 18;
            elbowConfig.stallCurentLimit = 20;
            elbowConfig.freeCurentLimit = 30;
            elbowConfig.kP = 0;
            elbowConfig.kI = 0;
            elbowConfig.kD = 0;
            elbowConfig.positionTolerance = 2;
            elbowConfig.minRPM = 0;
            elbowConfig.maxRPM = 0;
            elbowConfig.initialPosition = 0;
            elbowConfig.reduction = 100 * ( (24.0 / 12.0)*(46.0 / 24.0)) ;
            elbowConfig.subsystemName = "Elbow";
            
        ServoMotorSubsystem.ServoMotorSubsystemConfig wristConfig = new ServoMotorSubsystem.ServoMotorSubsystemConfig();
            wristConfig.motorCanID = 11;
            wristConfig.stallCurentLimit = 20;
            wristConfig.freeCurentLimit = 30;
            wristConfig.kP = 0.0002;
            wristConfig.kI = 0;
            wristConfig.kD = 0;
            wristConfig.positionTolerance = 2;
            wristConfig.minRPM = 600;
            wristConfig.maxRPM = 800;
            wristConfig.initialPosition = 0;
            wristConfig.reduction = 100 * (24.0 / 12.0);
            wristConfig.subsystemName = "Wrist";

        m_shoulderSubsystem = new ShoulderSubsystem(shoulderConfig);
        m_elbowSubsystem = new ElbowSubsystem(elbowConfig);
        m_wristSubsystem = new WristSubsystem(wristConfig);
        // m_shoulderSubsystem.setTarget(90);
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

        // cubeFloorIntakePosition.onTrue(new InstantCommand(() -> m_wristSubsystem.setTarget(0)));
        // coneFloorIntakePosition.onTrue(new InstantCommand(() -> m_wristSubsystem.setTarget(90)));

        // if(IntakeConstants.intakeRunning == false) {
        //     /*Run Intake- FORWARD */
        //     IntakeConstants.CONT_INTAKE_RUN.onTrue(new InstantCommand(() -> m_IntakeSubsystem.run()));
        //     /*Run Intake- REVERSE */
        //     IntakeConstants.CONT_INTAKE_RUN_REV.onTrue(new InstantCommand(() -> m_IntakeSubsystem.runReverse()));
        // } else if (IntakeConstants.intakeRunning == true) {
        //     /*Stop Intake- Both ways */
        //      IntakeConstants.CONT_INTAKE_RUN.onTrue(new InstantCommand(() -> m_IntakeSubsystem.stop()));
        //     IntakeConstants.CONT_INTAKE_RUN.onTrue(new InstantCommand(() -> m_IntakeSubsystem.stop()));
        // }
        //  IntakeConstants.CONT_INTAKE_RUN.onTrue(new InstantCommand(() -> m_IntakeSubsystem.run()));
        //  IntakeConstants.CONT_INTAKE_RUN.onFalse(new InstantCommand(() -> m_IntakeSubsystem.stop()));


        //  IntakeConstants.CONT_INTAKE_RUN_REV.onTrue(new InstantCommand(() -> m_IntakeSubsystem.runReverse()));
        //  IntakeConstants.CONT_INTAKE_RUN_REV.onFalse(new InstantCommand(() -> m_IntakeSubsystem.stop()));
    }

    public Command getAutonomousCommand() {
        return m_autoSelector.getSelected();
    }
}
