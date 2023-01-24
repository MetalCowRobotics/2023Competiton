

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StopMotorSubsystem;


public class StopMotorCommand extends CommandBase {
  // The subsystem the command runs on
  private final StopMotorSubsystem m_StopMotorSubsystem;

  public StopMotorCommand(StopMotorSubsystem subsystem) {
    m_StopMotorSubsystem = subsystem;
    addRequirements(m_StopMotorSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println(2);
  }

  @Override
  public void execute() {
    System.out.println("Your code is good-ish");
    m_StopMotorSubsystem.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}