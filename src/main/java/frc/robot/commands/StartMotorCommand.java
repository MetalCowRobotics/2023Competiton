

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StartMotorSubsystem;


public class StartMotorCommand extends CommandBase {
  // The subsystem the command runs on
  private final StartMotorSubsystem m_StartMotorSubsystem;

  public StartMotorCommand(StartMotorSubsystem subsystem) {
    m_StartMotorSubsystem = subsystem;
    addRequirements(m_StartMotorSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute(){
      System.out.println("Your code is good-ish");
      m_StartMotorSubsystem.startMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}