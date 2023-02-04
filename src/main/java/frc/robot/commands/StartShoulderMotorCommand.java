package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;


public class StartShoulderMotorCommand extends CommandBase {
  // The subsystem the command runs on
  private final ShoulderSubsystem m_ShoulderSubsystem;

  public StartShoulderMotorCommand(ShoulderSubsystem subsystem) {
    m_ShoulderSubsystem = subsystem;
    addRequirements(m_ShoulderSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_ShoulderSubsystem.startMotor();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}