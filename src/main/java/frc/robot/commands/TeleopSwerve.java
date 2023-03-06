package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier moveToCenter;
    private BooleanSupplier moveToLeft;
    private BooleanSupplier moveToRight;
    private BooleanSupplier autoLevel;

    PIDController anglePIDController = new PIDController(0.04, 0, 0.001);
    PIDController xController = new PIDController(.8, 0, 0);
    PIDController yController = new PIDController(.8, 0, 0);
    PIDController rollController = new PIDController(.6, 0, 0);

    private double targetX = -0.14;
    private double targetY = -0.42;
    private double tolerance = 0.03;

    // PIDController Controller = new PIDController(.04, 0, 0);

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier moveToCenter, BooleanSupplier moveToLeft, BooleanSupplier moveToRight, BooleanSupplier autoLevel) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.moveToCenter = moveToCenter;
        this.moveToLeft = moveToLeft;
        this.moveToRight = moveToRight;
        this.autoLevel = autoLevel;

        anglePIDController.setSetpoint(0);
        anglePIDController.setTolerance(6);

        xController.setSetpoint(targetX);
        yController.setSetpoint(targetY);

        rollController.setSetpoint(0);
        rollController.setTolerance(2, 2);
        
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/

        double x = s_Swerve.getPose().getX();
        double y = s_Swerve.getPose().getY();

        SmartDashboard.putNumber("tracked x", x);
        SmartDashboard.putNumber("tracked y", y);

        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(-rotationSup.getAsDouble(), Constants.stickDeadband);

        // 0.5, -0.9

        /* Level */
        if (autoLevel.getAsBoolean()) {
            double roll = s_Swerve.getRoll().getDegrees();

            roll = roll % 360;

            if (roll < 0) {
                roll += 360;
            }

            if (roll > 180) {
                anglePIDController.setSetpoint(360);
            } else {
                anglePIDController.setSetpoint(0);
            }

            double rotation = anglePIDController.calculate(roll);
            double Correction = xController.calculate(x);

            SmartDashboard.putNumber("absolute roll", roll);
            if (Math.abs(roll - 180) < 2) {
                rotation = 0;
                Correction = 0;
            }
            
            s_Swerve.drive(
                new Translation2d(Correction, 0).times(Constants.Swerve.maxSpeed), 
                -rotation * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                false
            );
        }

        /* Drive */
        if ((!moveToCenter.getAsBoolean() && !moveToLeft.getAsBoolean()) && (!moveToRight.getAsBoolean() && !autoLevel.getAsBoolean())) {
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                false
            );
        } else if (autoLevel.getAsBoolean()) {
            double roll = s_Swerve.getRoll().getDegrees();
            double correction = rollController.calculate(roll);

            if (rollController.atSetpoint()) {
                correction = 0.0;
            }

            s_Swerve.drive(
                new Translation2d(correction, 0).times(Constants.Swerve.maxSpeed / 4.0), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                false
            );
        } else {
            if (moveToCenter.getAsBoolean()) {
                targetX = -0.7;
                targetY = 0.00;
            }
            if (moveToRight.getAsBoolean()) {
                targetX = -0.7;
                targetY = -0.6;
            }
            if (moveToLeft.getAsBoolean()) {
                targetX = -0.7;
                targetY = 0.6;
                // System.out.println("MOVING LEFT");
            }

            xController.setSetpoint(targetX);
            yController.setSetpoint(targetY);

            System.out.println("tx:" + targetX + ", ty:" + targetY);
            double yaw = s_Swerve.getYaw().getDegrees();

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
            SmartDashboard.putNumber("absolute yaw", yaw);
            if (Math.abs(yaw - 180) < 2) {
                rotation = 0;
            }
            if (Math.abs(targetX-x) < tolerance) {
                xCorrection = 0;
            }
            if (Math.abs(targetY-y) < tolerance) {
                yCorrection = 0;
            }
            
            s_Swerve.drive(
                new Translation2d(xCorrection, yCorrection).times(Constants.Swerve.maxSpeed), 
                -rotation * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                false
            );
        }
        
    }
}