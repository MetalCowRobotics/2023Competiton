package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseIntakeMotor  extends CommandBase{
    
    private final IntakeSubsystem m_IntakeSubsystem;

    public ReverseIntakeMotor(IntakeSubsystem subsystem){
        m_IntakeSubsystem = subsystem;
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override 
    public void execute(){
        m_IntakeSubsystem.startMotorReverse();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
