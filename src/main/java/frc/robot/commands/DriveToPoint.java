package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveToPoint extends CommandBase {

    private Swerve m_swerve;

    private final double TOLERANCE = 0.05;
    private final double ANGLE_TOLERANCE = 0.05;

    private double targetX;
    private double targetY;
    private double targetAngle;

    private PIDController anglePIDController = new PIDController(0.04, 0, 0.001);
    private PIDController xController = new PIDController(0.8, 0, 0);
    private PIDController yController = new PIDController(0.8, 0, 0);

    public DriveToPoint(Swerve swerve, double x, double y, double theta) {
        this.m_swerve = swerve;
        addRequirements(m_swerve);

        anglePIDController.setSetpoint(0);
        anglePIDController.setTolerance(6);

        this.targetX = x;
        this.targetY = y;
        this.targetAngle = theta;

        xController.setSetpoint(targetX);
        yController.setSetpoint(targetY);
        anglePIDController.setSetpoint(targetAngle);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/

        double x = m_swerve.getPose().getX();
        double y = m_swerve.getPose().getY();

        SmartDashboard.putNumber("tracked x", x);
        SmartDashboard.putNumber("tracked y", y);
            
        xController.setSetpoint(targetX);
        yController.setSetpoint(targetY);

        // System.out.println("tx:" + targetX + ", ty:" + targetY);
        double yaw = m_swerve.getYaw().getDegrees();

        yaw = yaw % 360;
        if (yaw < 0) {
            yaw += 360;
        }

        if (targetAngle == 0) {
            if (yaw > 180) {
                anglePIDController.setSetpoint(360);
            } else {
                anglePIDController.setSetpoint(0);
            }
        }

        double rotation = anglePIDController.calculate(yaw);
        double xCorrection = xController.calculate(x);
        double yCorrection = yController.calculate(y);
        // SmartDashboard.putNumber("absolute yaw", yaw);
        
        m_swerve.drive(
            new Translation2d(xCorrection, yCorrection).times(Constants.Swerve.maxSpeed), 
            -rotation * Constants.Swerve.maxAngularVelocity, 
            true, 
            false
        );
    }

    @Override
    public boolean isFinished() {
        double x = m_swerve.getPose().getX();
        double y = m_swerve.getPose().getY();
        double yaw = m_swerve.getYaw().getDegrees();

        yaw = yaw % 360;
        if (yaw < 0) {
            yaw += 360;
        }

        if ( (Math.abs(x - targetX) < TOLERANCE && Math.abs(y - targetY) < TOLERANCE) && Math.abs(yaw - targetAngle) < ANGLE_TOLERANCE) {
            return true;
        }
        return false;
    }
}