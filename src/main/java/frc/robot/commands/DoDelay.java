package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DoDelay extends CommandBase {

    private double timeout;
    private Timer m_timer = new Timer();

    public DoDelay(double delay) {
        timeout = delay;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return (m_timer.hasElapsed(timeout));
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

}