package frc.robot.subsystem.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystem.drivetrain.enums.DriveType;
import frc.robot.subsystem.drivetrain.implementations.LimelightHelpers;
import frc.robot.subsystem.drivetrain.interfaces.IGyroInterface;
import frc.robot.subsystem.drivetrain.interfaces.IModuleInterface;
import frc.robot.utils.Logger;
import edu.wpi.first.wpilibj.Alert.AlertType;


public class DrivetrainSubsystem extends SubsystemBase {

    private final SwerveDrivePoseEstimator estimator;
    private final IGyroInterface gyro;
    private final IModuleInterface frontLeft, frontRight, backLeft, backRight;
    private final PPHolonomicDriveController driveController;

    Alert aprilTagAlert = new Alert("AprilTag", AlertType.kError);

    public DrivetrainSubsystem(
            IGyroInterface gyro,
            IModuleInterface frontLeft,
            IModuleInterface frontRight,
            IModuleInterface backLeft,
            IModuleInterface backRight) {

        this.gyro = gyro;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.estimator = new SwerveDrivePoseEstimator(
                DrivetrainConstants.KINEMATICS,
                gyro.getAngle(),
                getModulePositions(),
                RobotConstants.INITIAL_POSE);

        this.driveController = new PPHolonomicDriveController(
                AutoConstants.DRIVE_PID,
                AutoConstants.ANGLE_PID);
            
        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> this.drive(speeds),
                this.driveController,
                RobotConstants.ROBOT_CONFIG,
                () -> false,
                this);

        AutoBuilder.buildAutoChooser();
    }

    @Override
    public void periodic() {
        updateVisionOdometry();

        this.estimator.update(this.getRotation2d(), this.getModulePositions());

        // smartdashboarddan dereceler oku
        if(frontLeft instanceof frc.robot.subsystem.drivetrain.implementations.PhoenixModule){
            Logger.log("Drivetrain/RawAngles/FrontLeft", ((frc.robot.subsystem.drivetrain.implementations.PhoenixModule)frontLeft).getCANcoder().getDegrees());
            Logger.log("Drivetrain/RawAngles/FrontRight", ((frc.robot.subsystem.drivetrain.implementations.PhoenixModule)frontRight).getCANcoder().getDegrees());
            Logger.log("Drivetrain/RawAngles/BackLeft", ((frc.robot.subsystem.drivetrain.implementations.PhoenixModule)backLeft).getCANcoder().getDegrees());
            Logger.log("Drivetrain/RawAngles/BackRight", ((frc.robot.subsystem.drivetrain.implementations.PhoenixModule)backRight).getCANcoder().getDegrees());
        }
    }
    
    public void updateVisionOdometry() {
        boolean hasTarget = LimelightHelpers.getTV("limelight");// isim web arayüzündeki ile aynı olucak

        if (hasTarget) {
            Pose2d visionPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
            
            double latency = LimelightHelpers.getLatency_Pipeline("limelight") + LimelightHelpers.getLatency_Capture("limelight");
            double timestamp = Timer.getFPGATimestamp() - (latency / 1000.0);

            this.estimator.addVisionMeasurement(visionPose, timestamp);
        }
    }

    public Rotation2d getRotation2d() {
        return gyro.getAngle();
    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, RobotConstants.MAX_SPEED);
        this.frontLeft.setModuleState(moduleStates[0]);
        this.frontRight.setModuleState(moduleStates[1]);
        this.backLeft.setModuleState(moduleStates[2]);
        this.backRight.setModuleState(moduleStates[3]);

        Logger.log("Drivetrain/TestStates/ModuleStates", moduleStates);
        Logger.log("Drivetrain/TestStates/ModuleRealStates", this.getModuleStates());
    }

    public void drive(double xSpeed, double ySpeed, double rSpeed, DriveType driveType) {
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rSpeed);
        if (driveType == DriveType.RobotRelative) {
            this.drive(speeds);
        } else {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, this.getRotation2d());
            this.drive(speeds);
        }
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = DrivetrainConstants.KINEMATICS.toSwerveModuleStates(speeds);
        this.setModuleStates(moduleStates);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DrivetrainConstants.KINEMATICS.toChassisSpeeds(this.getModuleStates());
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                this.frontLeft.getModuleState(),
                this.frontRight.getModuleState(),
                this.backLeft.getModuleState(),
                this.backRight.getModuleState()
        };

    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                this.frontLeft.getModulePosition(),
                this.frontRight.getModulePosition(),
                this.backLeft.getModulePosition(),
                this.backRight.getModulePosition(),
        };
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        estimator.resetPose(pose);
    }

    public IModuleInterface[] getModules() {
        return new IModuleInterface[] { this.frontLeft, this.frontRight, this.backLeft, this.backRight };
    }
}