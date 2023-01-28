

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MotorSubsystem;


public class StopMotorCommand extends CommandBase {
  // The subsystem the command runs on
  private final MotorSubsystem m_MotorSubsystem;

  public StopMotorCommand(MotorSubsystem subsystem) {
    m_MotorSubsystem = subsystem;
    addRequirements(m_MotorSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_MotorSubsystem.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}