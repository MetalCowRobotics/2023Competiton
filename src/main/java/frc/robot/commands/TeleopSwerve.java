package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    PhotonCamera camera;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier moveToTargetPose) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        camera = new PhotonCamera("USB_Camera-B4.09.24.1");
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/

        PhotonPipelineResult result = camera.getLatestResult();
        // System.out.println(result.getBestTarget().getFiducialId());
        // result.getBestTarget = target = result..BestTarget 
        PhotonTrackedTarget target = result.getBestTarget();
        if (null != target) {
            Transform3d targetPose = target.getBestCameraToTarget();
            Translation2d xyPosition = new Translation2d(targetPose.getX(), targetPose.getY());

            double yaw = s_Swerve.getYaw().getDegrees();
            double correctionAngle = 0;
            if (yaw <= 90) {
                correctionAngle = 90 - yaw;
            } else if (yaw <= 180) {
                correctionAngle = 180 - yaw;
            } else if (yaw <= 270) {
                correctionAngle = 270 - yaw;
            } else if (yaw <= 360) {
                correctionAngle = 360 - yaw;
            }

            Rotation2d correction = Rotation2d.fromDegrees(-correctionAngle);
            xyPosition = xyPosition.rotateBy(correction);

            SmartDashboard.putNumber("tracked x", xyPosition.getX() * 39.37);
            SmartDashboard.putNumber("tracked y", xyPosition.getY() * 39.37);
    
        }

        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(-rotationSup.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            false
        );
    }
}