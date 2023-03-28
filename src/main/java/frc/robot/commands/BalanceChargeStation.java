package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class BalanceChargeStation extends CommandBase {
    
    private Swerve m_swerve;
    private PIDController chargeStationAngleController = new PIDController(0.004, 0.0, 0.0);

    private double chargeStationAngle;

    private double tolerance = 2;

    public BalanceChargeStation(Swerve swerve) {
        this.m_swerve = swerve;

        chargeStationAngleController.setTolerance(tolerance);

        addRequirements(m_swerve);
    }

    @Override
    public void execute() {
        chargeStationAngle = m_swerve.getBalanceAngle().getDegrees();
        double xComponent = chargeStationAngleController.calculate(chargeStationAngle);

        SmartDashboard.putNumber("balancing output", xComponent);

        m_swerve.driveAuto(
            new Translation2d(xComponent, 0).times(Constants.Swerve.maxSpeed), 
            0, 
            true, 
            false
        );
    }

    @Override
    public boolean isFinished() {
        return chargeStationAngleController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.driveAuto(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            0, 
            true, 
            false
        );
    }

}
