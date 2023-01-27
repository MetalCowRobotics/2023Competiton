// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final TalonSRXControlMode TalonSRXControlMode = null;

  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    //m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */


  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  
DigitalInput input = new DigitalInput(9);
// DigitalInput num = new DigitalInput(8)

XboxController xbox = new XboxController(0);
TalonSRX motor = new TalonSRX(16);
TalonSRX motor2 = new TalonSRX(15);
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //System.out.println(analog);
    if (xbox.getRawButtonPressed( 1)) {
      motor.set(com.ctre.phoenix.motorcontrol.TalonSRXControlMode.PercentOutput, 0);
      motor2.set(com.ctre.phoenix.motorcontrol.TalonSRXControlMode.PercentOutput, 0);
    
  }
    if (xbox.getRawButtonReleased(1) || (input.get() == false)) {
      motor.set(com.ctre.phoenix.motorcontrol.TalonSRXControlMode.PercentOutput, 0.50);
      motor2.set(com.ctre.phoenix.motorcontrol.TalonSRXControlMode.PercentOutput, 0.50);  

    }
    // if (input.get()) {
    //   
    // }
    // else {
    //   System.out.println("BYE");
    //     motor.set(com.ctre.phoenix.motorcontrol.TalonSRXControlMode.PercentOutput, 0);
    //     motor2.set(com.ctre.phoenix.motorcontrol.TalonSRXControlMode.PercentOutput, 0);

      
    

  }
}

