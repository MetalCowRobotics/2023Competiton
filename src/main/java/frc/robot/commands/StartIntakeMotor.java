package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class StartIntakeMotor  extends CommandBase{
    
    private final IntakeSubsystem m_IntakeSubsystem;

    public StartIntakeMotor(IntakeSubsystem subsystem){
        m_IntakeSubsystem = subsystem;
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override 
    public void execute(){
        m_IntakeSubsystem.startMotor();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
