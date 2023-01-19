// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import frc.robot.subsystems.StartMotorSubsystem;
import frc.robot.commands.StartMotorCommand;
import frc.robot.subsystems.StopMotorSubsystem;
import frc.robot.commands.StopMotorCommand;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    // m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // m_robotContainer.startTeleop();
  }

 // motor.set(TalonSRXControlMode.PercentOutput, 0.5);
  StartMotorSubsystem startSub = new StartMotorSubsystem();
  StartMotorCommand startCom = new StartMotorCommand(startSub);
  StopMotorSubsystem stopSub = new StopMotorSubsystem();
  StopMotorCommand stopCom = new StopMotorCommand(stopSub);
  DigitalInput coolSensor = new DigitalInput(6);


  @Override
  public void teleopPeriodic() {
    System.out.println("Your code sucks");
    if (coolSensor.get()) {
      CommandScheduler.getInstance().schedule(startCom);
      System.out.println("Your code is kinda sketchy");
    } else {
      CommandScheduler.getInstance().schedule(stopCom);
      System.out.println("Your code is kinda sketchy");
    }
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {

  }
}
