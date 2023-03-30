package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AlignToSubstation extends CommandBase {
    
    private Swerve m_swerve;

    private PIDController anglePIDController = new PIDController(0.004, 0, 0.000);
    private PIDController xController = new PIDController(.8, 0, 0);
    private PIDController yController = new PIDController(.8, 0, 0);

    double targetX;
    double targetY;
    double targetYaw = 0;

    double x;
    double y;
    double yaw;

    private double tolerance = 0.03;

    public AlignToSubstation(Swerve swerve, double targetX, double targetY) {
        this.m_swerve = swerve;
        this.targetX = targetX;
        this.targetY = targetY;
        addRequirements(m_swerve);
    }

    @Override
    public void execute() {
        x = m_swerve.getPose().getX();
        y = m_swerve.getPose().getY();

        xController.setSetpoint(targetX);
        yController.setSetpoint(targetY);
        anglePIDController.setSetpoint(targetYaw);
        anglePIDController.setTolerance(1, 1);

        yaw = m_swerve.getYaw().getDegrees();

        yaw = yaw % 360;

        if (yaw < 0) {
            yaw += 360;
        }

        if (yaw > 180) {
            anglePIDController.setSetpoint(360);
        } else {
            anglePIDController.setSetpoint(0);
        }

        double rotation = anglePIDController.calculate(yaw);
        double xCorrection = xController.calculate(x);
        double yCorrection = yController.calculate(y);

        if (anglePIDController.atSetpoint()) {
            rotation = 0;
        }
        if (Math.abs(targetX - x) < tolerance) {
            xCorrection = 0;
        }
        if (Math.abs(targetY - y) < tolerance) {
            yCorrection = 0;
        }
        
        m_swerve.drive(
            new Translation2d(xCorrection, yCorrection).times(Constants.Swerve.maxSpeed), 
            -rotation * Constants.Swerve.maxAngularVelocity, 
            true, 
            false
        );
    }

    @Override
    public boolean isFinished() {
        if (anglePIDController.atSetpoint()) {
            if (Math.abs(targetX - x) < tolerance) {
                if (Math.abs(targetY - y) < tolerance) {
                    return true;
                }
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
            true, 
            false
        );
    }
}