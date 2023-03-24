package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class DisableVision extends CommandBase {
    
    private Swerve m_swerve;

    public DisableVision(Swerve swerve) {
        this.m_swerve = swerve;
        addRequirements(m_swerve);
    }

    @Override
    public void execute() {
        m_swerve.disableVision();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
