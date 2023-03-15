package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private boolean visionEnabled = true;

    private SwerveDrivePoseEstimator estimator;
    PhotonCamera camera;
    double accelerationTime = 0.5;
    int i = 0;
    private double linearAcceleration = Constants.Swerve.maxSpeed / accelerationTime;
    private double angularAcceleration = Constants.Swerve.maxAngularVelocity / accelerationTime;

    private SlewRateLimiter m_xSlewRateLimiter = new SlewRateLimiter(linearAcceleration, -linearAcceleration, 0);
    private SlewRateLimiter m_ySlewRateLimiter = new SlewRateLimiter(linearAcceleration, -linearAcceleration, 0);
    private SlewRateLimiter m_angleSlewRateLimiter = new SlewRateLimiter(angularAcceleration, -angularAcceleration, 0);

    private int lastTrackedTarget = -1;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        estimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), new Pose2d(0, 0, getYaw()));

        camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
        
    }

    public void disableVision() {
        visionEnabled = false;
    }

    public void enableVision() {
        visionEnabled = true;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        double xSpeed = m_xSlewRateLimiter.calculate(translation.getX());
        double ySpeed = m_ySlewRateLimiter.calculate(translation.getY());
        double angularVelocity = m_angleSlewRateLimiter.calculate(rotation);
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    xSpeed, 
                                    ySpeed, 
                                    angularVelocity, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    xSpeed, 
                                    ySpeed, 
                                    angularVelocity
                                )
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }
    
    public void driveAuto(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        double xSpeed = translation.getX();
        double ySpeed = translation.getY();
        double angularVelocity = rotation;
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    xSpeed, 
                                    ySpeed, 
                                    angularVelocity, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    xSpeed, 
                                    ySpeed, 
                                    angularVelocity
                                )
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        estimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public void zeroGyro(double angle){
        gyro.setYaw(angle);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Rotation2d getRoll() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getRoll()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    private void addVisionMeasurement() {
        PhotonPipelineResult result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        if (null != target) {
            double time = result.getTimestampSeconds();
            Transform3d targetPose = target.getBestCameraToTarget();
            Translation2d xyPosition = new Translation2d(targetPose.getX(), targetPose.getY());

            double yaw = getYaw().getDegrees();
            yaw = yaw % 360;
            if (yaw < 0) {
                yaw += 360;
            }
            double correctionAngle = 0;
            if (yaw <= 90) {
                correctionAngle = 90 - yaw;
            } else if (yaw <= 180) {
                correctionAngle = 180 - yaw;
            } else if (yaw <= 270) {
                correctionAngle = 270 - yaw;
            } else if (yaw <= 360) {
                correctionAngle = 360 - yaw; // 360 - yaw + 90
            }

            Rotation2d correction = Rotation2d.fromDegrees(-correctionAngle);
            xyPosition = xyPosition.rotateBy(correction);
            
            // if (yaw < 180) {
            //     xyPosition = new Translation2d(xyPosition.getY(), xyPosition.getX());
            // }
            Pose2d pose = new Pose2d(xyPosition.getY() + Units.inchesToMeters(10.5), -xyPosition.getX() - Units.inchesToMeters(4.25), getYaw());
            if (lastTrackedTarget == target.getFiducialId()) {
                estimator.addVisionMeasurement(pose, time);
            } else {
                resetOdometry(pose);
            }
            lastTrackedTarget = target.getFiducialId();
            SmartDashboard.putNumber("x from vision", pose.getX());
            SmartDashboard.putNumber("y from vision", pose.getY());
        }
        SmartDashboard.putNumber("x from vision", -99);
        SmartDashboard.putNumber("y from vision", -99);
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());
        estimator.update(getYaw(), getModulePositions());
        if (visionEnabled) {
            SmartDashboard.putNumber("vision count", i);
            i++;
            addVisionMeasurement();
        }
        SmartDashboard.putBoolean("vision enabled", visionEnabled);

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        SmartDashboard.putNumber("yaw", getYaw().getDegrees());
        Pose2d pose = swerveOdometry.getPoseMeters();
        SmartDashboard.putNumber("x from odometry", pose.getX());
        SmartDashboard.putNumber("y from odometry", pose.getY());
    }
}