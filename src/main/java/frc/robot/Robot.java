package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  DigitalInput garageSensor = new DigitalInput(0);
  TalonSRX motor1 = new TalonSRX(11);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

  }

  /** This function is called periodically when the robot is on. */
  @Override
  public void robotPeriodic() {
    
  }

  /** This function is called once at the start of Disabled mode. */
  @Override
  public void disabledInit() {

  }

  /** This function is called periodically during Disable mode. */
  @Override
  public void disabledPeriodic() {

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once at the start of operator control. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if(garageSensor.get() == true) {
      SmartDashboard.putString("Title", "True");
      motor1.set(TalonSRXControlMode.PercentOutput, 0.3);

    }
    else {
      SmartDashboard.putString("Title", "False");
      motor1.set(TalonSRXControlMode.PercentOutput, 0);
      
    }

    // Link to the Analog UltraSonicSensor Class. Find the method to return distance and impliment it.
    // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/AnalogPotentiometer.html#method.summary
    // Similar work of creating an object like the infrared sensor. 

  }

  /** This function is called once at the start of test mode. */
  @Override
  public void testInit() {
    
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }
}
