package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToPoint;
import frc.robot.commands.ArmToAngles;
import frc.robot.commands.DisableVision;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.EnableVision;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ToggleColor;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
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
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton moveToCenter = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton moveToLeft = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton moveToRight = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton autoLevel = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    
    /* Operator Buttons */
    private final JoystickButton cubeSubstationIntakePosition = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton coneSubstationIntakePosition = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton cubeFloorIntakePosition = new JoystickButton(operator, XboxController.Axis.kLeftY);
    // private final JoystickButton coneFloorIntakePosition = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton lowScoringPosition = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton midScoringPosition = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton stow = new JoystickButton(operator, XboxController.Button.kA.value);

    Trigger crawl = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.8);
    Trigger sprint = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.8);

    Trigger cubeFloorIntakePosition = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightY.value) > 0.8);
    Trigger coneFloorIntakePosition = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kLeftY.value) > 0.8);

    Trigger intakeForward = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.7);
    Trigger intakeReverse = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.7);
    Trigger stopIntake = new Trigger(() -> operator.getRawButtonPressed(XboxController.Button.kX.value));

    Trigger substationRight = new Trigger(() -> driver.getRawButtonPressed(XboxController.Button.kRightBumper.value));
    Trigger substationLeft = new Trigger(() -> driver.getRawButtonPressed(XboxController.Button.kLeftBumper.value));

    Trigger wristUp = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kLeftX.value) > 0.7);
    Trigger wristDown = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightX.value) > 0.7);

    Trigger shootHigh = new Trigger(() -> operator.getRawButtonPressed(XboxController.Button.kStart.value));
    Trigger toggleLED = new Trigger(() -> driver.getRawButtonPressed(XboxController.Button.kY.value));

    Trigger drive = new Trigger(() -> 
        (Math.abs(driver.getRawAxis(XboxController.Axis.kLeftX.value)) > 0.1 || Math.abs(driver.getRawAxis(XboxController.Axis.kLeftY.value)) > 0.1) || 
        (Math.abs(driver.getRawAxis(XboxController.Axis.kRightX.value)) > 0.1 || Math.abs(driver.getRawAxis(XboxController.Axis.kRightY.value)) > 0.1)
    );
    // private final JoystickButton stopstow = new JoystickButton(operator, XboxController.Button.kB.value);

    /* Subsystems */
    private Swerve m_swerve = new Swerve();
    private ShoulderSubsystem m_shoulderSubsystem;
    private ElbowSubsystem m_elbowSubsystem;
    private WristSubsystem m_wristSubsystem;
    private IntakeSubsystem m_IntakeSubsystem;
    private LEDSubsystem m_LEDSubsystem;
    /* Autos */
    private double armMovementTimeout = 4;
    private SendableChooser<Command> m_autoSelector;               
    
    private Command chargeStationScoreMobilityDock;
    private Command chargeStationScoreDock;
    private Command substationScoreMobilityDockBlue;
    private Command substationScoreMobilityDockRed;
    private Command substationScoreMobilityBlue;
    private Command cableRunScoreMobility;
    private Command armTest;
    
    private Command alignToMiddle;
    private Command alignToLeft;
    private Command alignToRight;
    private Command alignToSubstationRight;
    private Command changeColor;

    private Command noAuto = new InstantCommand(() -> m_swerve.zeroGyro(180));


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        m_swerve = new Swerve();
        m_LEDSubsystem = new LEDSubsystem(1);
        m_IntakeSubsystem = new IntakeSubsystem();
        m_autoSelector = new SendableChooser<Command>();

        ServoMotorSubsystem.ServoMotorSubsystemConfig shoulderConfig = new ServoMotorSubsystem.ServoMotorSubsystemConfig();
            shoulderConfig.motorCanID = 15;
            shoulderConfig.stallCurentLimit = 30;
            shoulderConfig.freeCurentLimit = 30;
            shoulderConfig.rampTime = 0.25;
            shoulderConfig.kP = 0.07;
            shoulderConfig.kI = 0.0;
            shoulderConfig.kD = 0.0;
            shoulderConfig.positionTolerance = 4;
            shoulderConfig.rampTime = 0;
            shoulderConfig.minRPM = 2000;
            shoulderConfig.maxRPM = 5200;
            shoulderConfig.initialPosition = 0;
            shoulderConfig.reduction = 100 * (46.0 / 12.0);
            shoulderConfig.subsystemName = "Shoulder";

        ServoMotorSubsystem.ServoMotorSubsystemConfig elbowConfig = new ServoMotorSubsystem.ServoMotorSubsystemConfig();
            elbowConfig.motorCanID = 18;
            elbowConfig.stallCurentLimit = 20;
            elbowConfig.freeCurentLimit = 30;
            elbowConfig.kP = 0.02;
            elbowConfig.kI = 0.0;
            elbowConfig.kD = 0;
            elbowConfig.positionTolerance = 2;
            elbowConfig.minRPM = 2000;
            elbowConfig.maxRPM = 3500;
            elbowConfig.initialPosition = 0;
            elbowConfig.reduction = 100 * ( (24.0 / 12.0) * (46.0 / 24.0) );
            elbowConfig.subsystemName = "Elbow";
            
        ServoMotorSubsystem.ServoMotorSubsystemConfig wristConfig = new ServoMotorSubsystem.ServoMotorSubsystemConfig();
            wristConfig.motorCanID = 19;
            wristConfig.stallCurentLimit = 20;
            wristConfig.freeCurentLimit = 30;
            wristConfig.kP = 0.0002;
            wristConfig.kI = 0;
            wristConfig.kD = 0;
            wristConfig.positionTolerance = 2;
            wristConfig.minRPM = 1800;
            wristConfig.maxRPM = 2000;
            wristConfig.initialPosition = 0;
            wristConfig.reduction = 100 * (24.0 / 12.0);
            wristConfig.subsystemName = "Wrist";

        m_shoulderSubsystem = new ShoulderSubsystem(shoulderConfig);
        m_elbowSubsystem = new ElbowSubsystem(elbowConfig);
        m_wristSubsystem = new WristSubsystem(wristConfig);

        cableRunScoreMobility = new SequentialCommandGroup( // added arm, works
            new DisableVision(m_swerve),
            new InstantCommand(() -> m_swerve.zeroGyro(180)),
            new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0, 0, m_swerve.getYaw()))),
            new ParallelRaceGroup(
                new ArmToAngles(m_wristSubsystem, m_elbowSubsystem, m_shoulderSubsystem, Constants.ArmConstants.MidScoring.SHOULDER_ANGLE, Constants.ArmConstants.MidScoring.ELBOW_ANGLE,Constants.ArmConstants.MidScoring.WRIST_ANGLE),
                new WaitCommand(armMovementTimeout)
            ),
            new InstantCommand(() -> m_IntakeSubsystem.runReverse()),
            new WaitCommand(0.5),
            new InstantCommand(() -> m_IntakeSubsystem.stop()),
            new ParallelRaceGroup(
                new ArmToAngles(m_wristSubsystem, m_elbowSubsystem, m_shoulderSubsystem, 0, 0, 0),
                new WaitCommand(armMovementTimeout)
            ),
            new DriveToPoint(m_swerve, -4, 0, 180),
            new EnableVision(m_swerve)
        );

        chargeStationScoreMobilityDock = new SequentialCommandGroup(
            new DisableVision(m_swerve),
            new InstantCommand(() -> m_swerve.zeroGyro(180)),
            new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0, 0, m_swerve.getYaw()))),
            new ParallelRaceGroup(
                new ArmToAngles(m_wristSubsystem, m_elbowSubsystem, m_shoulderSubsystem, Constants.ArmConstants.MidScoring.SHOULDER_ANGLE, Constants.ArmConstants.MidScoring.ELBOW_ANGLE,Constants.ArmConstants.MidScoring.WRIST_ANGLE),
                new WaitCommand(armMovementTimeout)
            ),
            new InstantCommand(() -> m_IntakeSubsystem.runReverse()),
            new WaitCommand(0.5),
            new InstantCommand(() -> m_IntakeSubsystem.stop()),
            new ParallelRaceGroup(
                new ArmToAngles(m_wristSubsystem, m_elbowSubsystem, m_shoulderSubsystem, 0, 0, 0),
                new WaitCommand(armMovementTimeout)
            ),
            new DriveToPoint(m_swerve, -5.5, 0, 180),
            new DriveToPoint(m_swerve, -2.715, 0, 180),
            new EnableVision(m_swerve)
        );

        substationScoreMobilityBlue = new SequentialCommandGroup( // added arm, works
            new DisableVision(m_swerve),
            new InstantCommand(() -> m_swerve.zeroGyro(180)),
            new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0, 0, m_swerve.getYaw()))),
            new ParallelRaceGroup(
                new ArmToAngles(m_wristSubsystem, m_elbowSubsystem, m_shoulderSubsystem, Constants.ArmConstants.MidScoring.SHOULDER_ANGLE, Constants.ArmConstants.MidScoring.ELBOW_ANGLE,Constants.ArmConstants.MidScoring.WRIST_ANGLE),
                new WaitCommand(armMovementTimeout)
            ),
            new InstantCommand(() -> m_IntakeSubsystem.runReverse()),
            new WaitCommand(0.5),
            new InstantCommand(() -> m_IntakeSubsystem.stop()),
            new ParallelRaceGroup(
                new ArmToAngles(m_wristSubsystem, m_elbowSubsystem, m_shoulderSubsystem, 0, 0, 0),
                new WaitCommand(armMovementTimeout)
            ),
            new DriveToPoint(m_swerve, -3, 0, 180),
            new EnableVision(m_swerve)
        );

        substationScoreMobilityDockBlue = new SequentialCommandGroup(
            new DisableVision(m_swerve),
            new InstantCommand(() -> m_swerve.zeroGyro(180)),
            new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0, 0, m_swerve.getYaw()))),
            new ParallelRaceGroup(
                new ArmToAngles(m_wristSubsystem, m_elbowSubsystem, m_shoulderSubsystem, Constants.ArmConstants.MidScoring.SHOULDER_ANGLE, Constants.ArmConstants.MidScoring.ELBOW_ANGLE,Constants.ArmConstants.MidScoring.WRIST_ANGLE),
                new WaitCommand(armMovementTimeout)
            ),
            new InstantCommand(() -> m_IntakeSubsystem.runReverse()),
            new WaitCommand(0.5),
            new InstantCommand(() -> m_IntakeSubsystem.stop()),
            new ParallelRaceGroup(
                new ArmToAngles(m_wristSubsystem, m_elbowSubsystem, m_shoulderSubsystem, 0, 0, 0),
                new WaitCommand(armMovementTimeout)
            ),
            new DriveToPoint(m_swerve, -4.3, 0, 180),
            new DriveToPoint(m_swerve, -4.3, 2.5, 180),
            new DriveToPoint(m_swerve, -2.1, 2.5, 180),
            new EnableVision(m_swerve)
        );

        substationScoreMobilityDockRed = new SequentialCommandGroup(
            new DisableVision(m_swerve),
            new InstantCommand(() -> m_swerve.zeroGyro(180)),
            new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0, 0, m_swerve.getYaw()))),
            new ParallelRaceGroup(
                new ArmToAngles(m_wristSubsystem, m_elbowSubsystem, m_shoulderSubsystem, Constants.ArmConstants.MidScoring.SHOULDER_ANGLE, Constants.ArmConstants.MidScoring.ELBOW_ANGLE,Constants.ArmConstants.MidScoring.WRIST_ANGLE),
                new WaitCommand(armMovementTimeout)
            ),
            new InstantCommand(() -> m_IntakeSubsystem.runReverse()),
            new WaitCommand(0.5),
            new InstantCommand(() -> m_IntakeSubsystem.stop()),
            new ParallelRaceGroup(
                new ArmToAngles(m_wristSubsystem, m_elbowSubsystem, m_shoulderSubsystem, 0, 0, 0),
                new WaitCommand(armMovementTimeout)
            ),
            new DriveToPoint(m_swerve, -4.3, 0, 180),
            new DriveToPoint(m_swerve, -4.3, -2.5, 180),
            new DriveToPoint(m_swerve, -2.1, -2.5, 180),
            new EnableVision(m_swerve)
        );

        chargeStationScoreDock = new SequentialCommandGroup(
            new DisableVision(m_swerve),
            new InstantCommand(() -> m_swerve.zeroGyro(180)),
            new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0, 0, m_swerve.getYaw()))),
            new ParallelRaceGroup(
                new ArmToAngles(m_wristSubsystem, m_elbowSubsystem, m_shoulderSubsystem, Constants.ArmConstants.MidScoring.SHOULDER_ANGLE, Constants.ArmConstants.MidScoring.ELBOW_ANGLE,Constants.ArmConstants.MidScoring.WRIST_ANGLE),
                new WaitCommand(armMovementTimeout)
            ),
            new InstantCommand(() -> m_IntakeSubsystem.runReverse()),
            new WaitCommand(0.5),
            new InstantCommand(() -> m_IntakeSubsystem.stop()),
            new ParallelRaceGroup(
                new ArmToAngles(m_wristSubsystem, m_elbowSubsystem, m_shoulderSubsystem, 0, 0, 0),
                new WaitCommand(armMovementTimeout)
            ),
            new DriveToPoint(m_swerve, -2.7, 0, 180),
            new EnableVision(m_swerve)
        );
        if (DriverStation.getAlliance().equals(Alliance.Blue)) {
            alignToMiddle = new AlignToPoint(m_swerve, -0.4, 0, 180);
            alignToLeft = new AlignToPoint(m_swerve, -0.4, 0.577, 180);
            alignToRight = new AlignToPoint(m_swerve, -0.4, -0.577-0.15, 180);
        } else {
            alignToMiddle = new AlignToPoint(m_swerve, -0.4, 0, 180);
            alignToLeft = new AlignToPoint(m_swerve, -0.4, -0.577, 180);
            alignToRight = new AlignToPoint(m_swerve, -0.4, 0.577+0.15, 180);
        }
        
        m_autoSelector.addOption("Charge Station Score + Dock", chargeStationScoreDock);
        m_autoSelector.addOption("Charge Station Score + Mobility + Dock", chargeStationScoreMobilityDock);
        m_autoSelector.addOption("Blue Substation Score + Mobility + Dock", substationScoreMobilityDockBlue);
        m_autoSelector.addOption("Red Substation Score + Mobility + Dock", substationScoreMobilityDockRed);
        m_autoSelector.addOption("Blue Substation Score + Mobility", substationScoreMobilityBlue);
        m_autoSelector.addOption("Cable Run Score Mobility", cableRunScoreMobility);
        m_autoSelector.addOption("arm test", armTest);
        m_autoSelector.setDefaultOption("None", noAuto);
        SmartDashboard.putData(m_autoSelector);        
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
                () -> autoLevel.getAsBoolean()
            )
        );
        

        zeroGyro.onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));
        moveToCenter.onTrue(alignToMiddle);
        moveToLeft.onTrue(alignToLeft);
        moveToRight.onTrue(alignToRight);
        changeColor = new ToggleColor(m_LEDSubsystem);
        toggleLED.onTrue(changeColor);

        drive.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancel(alignToLeft, alignToMiddle, alignToRight)));

        m_swerve.enableVision();

        intakeForward.onTrue(new InstantCommand(() -> m_IntakeSubsystem.run()));
        intakeForward.onFalse(new InstantCommand(() -> m_IntakeSubsystem.stop()));

        intakeReverse.onTrue(new InstantCommand(() -> m_IntakeSubsystem.runReverse()));
        intakeReverse.onFalse(new InstantCommand(() -> m_IntakeSubsystem.stop()));

        stopIntake.onTrue(new InstantCommand(() -> m_IntakeSubsystem.stop()));

        wristUp.onTrue(new InstantCommand(() -> m_wristSubsystem.wristUp()));
        wristDown.onTrue(new InstantCommand(() -> m_wristSubsystem.wristDown()));

        crawl.onTrue(new InstantCommand(() -> m_swerve.setCrawl()));
        crawl.onFalse(new InstantCommand(() -> m_swerve.setBase()));

        sprint.onTrue(new InstantCommand(() -> m_swerve.setSprint()));
        sprint.onFalse(new InstantCommand(() -> m_swerve.setBase()));


        midScoringPosition.onTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> m_shoulderSubsystem.setTarget(Constants.ArmConstants.MidScoring.SHOULDER_ANGLE)
                ),
                new InstantCommand(
                    () -> m_elbowSubsystem.setTarget(Constants.ArmConstants.MidScoring.ELBOW_ANGLE)
                ), 
                new InstantCommand(
                    () -> m_wristSubsystem.setTarget(Constants.ArmConstants.MidScoring.WRIST_ANGLE)
                ),
                new InstantCommand(
                    () -> m_IntakeSubsystem.stop()
                )
            )
        );
        coneFloorIntakePosition.onTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> m_shoulderSubsystem.setTarget(Constants.ArmConstants.GroundCone.SHOULDER_ANGLE)
                ),
                new InstantCommand(
                    () -> m_elbowSubsystem.setTarget(Constants.ArmConstants.GroundCone.ELBOW_ANGLE)
                ), 
                new InstantCommand(
                    () -> m_wristSubsystem.setTarget(Constants.ArmConstants.GroundCone.WRIST_ANGLE)
                ),
                new InstantCommand(
                    () -> m_IntakeSubsystem.runReverse()
                )
            )
        );
        cubeFloorIntakePosition.onTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> m_shoulderSubsystem.setTarget(Constants.ArmConstants.GroundCube.SHOULDER_ANGLE)
                ),
                new InstantCommand(
                    () -> m_elbowSubsystem.setTarget(Constants.ArmConstants.GroundCube.ELBOW_ANGLE)
                ), 
                new InstantCommand(
                    () -> m_wristSubsystem.setTarget(Constants.ArmConstants.GroundCube.WRIST_ANGLE)
                ),
                new InstantCommand(
                    () -> m_IntakeSubsystem.run()
                )
            )
        );
        lowScoringPosition.onTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> m_shoulderSubsystem.setTarget(Constants.ArmConstants.LowScoring.SHOULDER_ANGLE)
                ),
                new InstantCommand(
                    () -> m_elbowSubsystem.setTarget(Constants.ArmConstants.LowScoring.ELBOW_ANGLE)
                ), 
                new InstantCommand(
                    () -> m_wristSubsystem.setTarget(Constants.ArmConstants.LowScoring.WRIST_ANGLE)
                ), 
                new InstantCommand(
                    () -> m_IntakeSubsystem.stop()
                )
            )
        );
        coneSubstationIntakePosition.onTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> m_shoulderSubsystem.setTarget(Constants.ArmConstants.SubstationCone.SHOULDER_ANGLE)
                ),
                new InstantCommand(
                    () -> m_elbowSubsystem.setTarget(Constants.ArmConstants.SubstationCone.ELBOW_ANGLE)
                ), 
                new InstantCommand(
                    () -> m_wristSubsystem.setTarget(Constants.ArmConstants.SubstationCone.WRIST_ANGLE)
                ), 
                new InstantCommand(
                    () -> m_IntakeSubsystem.runReverse()
                )
            )
        );
        cubeSubstationIntakePosition.onTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> m_shoulderSubsystem.setTarget(Constants.ArmConstants.SubstationCube.SHOULDER_ANGLE)
                ),
                new InstantCommand(
                    () -> m_elbowSubsystem.setTarget(Constants.ArmConstants.SubstationCube.ELBOW_ANGLE)
                ), 
                new InstantCommand(
                    () -> m_wristSubsystem.setTarget(Constants.ArmConstants.SubstationCube.WRIST_ANGLE)
                ), 
                new InstantCommand(
                    () -> m_IntakeSubsystem.run()
                )
            )
        );
        stow.onTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> m_shoulderSubsystem.setTarget(0)
                ),
                new InstantCommand(
                    () -> m_elbowSubsystem.setTarget(0)
                ), 
                new InstantCommand(
                    () -> m_wristSubsystem.setTarget(0)
                ), 
                new InstantCommand(
                    () -> m_IntakeSubsystem.stop()
                )
            )
        );
        m_IntakeSubsystem.stop();

        shootHigh.onTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> m_shoulderSubsystem.setTarget(Constants.ArmConstants.HighScoring.SHOULDER_ANGLE)
                ),
                new InstantCommand(
                    () -> m_elbowSubsystem.setTarget(Constants.ArmConstants.HighScoring.ELBOW_ANGLE)
                ), 
                new InstantCommand(
                    () -> m_wristSubsystem.setTarget(Constants.ArmConstants.HighScoring.WRIST_ANGLE)
                )
            )
        );
    }

    public Command getAutonomousCommand() {
        CommandScheduler.getInstance().removeDefaultCommand(m_swerve);
        return m_autoSelector.getSelected();
    }
}
