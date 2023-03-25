package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase{
    
    private Swerve m_Swerve;
    private BooleanSupplier robotCentricSup;

    private PIDController rollPIDController = new PIDController(0.6, 0, 0);
    private PIDController drivePIDController = new PIDController(.8,0,0);

    private double target = 0;

    public AutoBalance(Swerve m_Swerve, BooleanSupplier robotCentricSup) {
        this.m_Swerve = m_Swerve;
        addRequirements(m_Swerve);
        this.robotCentricSup = robotCentricSup;

        rollPIDController.setSetpoint(0);

        drivePIDController.setSetpoint(target);
    }

    @Override
    public void execute() {

        double x = m_Swerve.getPose().getX();
        double roll = m_Swerve.getRoll().getDegrees();

        roll = roll % 360;

        if (roll < 0) {
            roll += 360;
        }

        if (roll > 180) {
            rollPIDController.setSetpoint(360);
        } else {
            rollPIDController.setSetpoint(0);
        }

        double rotation = rollPIDController.calculate(roll);
        double Correction = drivePIDController.calculate(x);

        SmartDashboard.putNumber("absolute roll", roll);
        if (Math.abs(roll - 180) < 2) {
            rotation = 0;
            Correction = 0;
        }
        
        m_Swerve.drive(
            new Translation2d(Correction, 0).times(Constants.Swerve.maxSpeed), 
            -rotation * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            false
        );
    }
}