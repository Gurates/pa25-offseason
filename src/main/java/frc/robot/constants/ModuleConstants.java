package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import frc.robot.utils.COTSTalonFXSwerveConstants;


public class ModuleConstants {

    public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
    COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

    public static final Current DRIVE_CURRENT_LIMIT = Amps.of(60);
    public static final Current ANGLE_CURRENT_LIMIT = Amps.of(40);

    public static final int ANGLE_CURRENT_THRESHOLD = 40;
    public static final double ANGLE_CURRENT_THRESHOLD_TIME = 0.1;
    public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

    public static final int DRIVE_CURRENT_THRESHOLD = 60;
    public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
    public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

    //pıd
    public static final double DRIVE_P = 0.7; // titreme
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;
    public static final double DRIVE_IZ = 0.0;
    public static final double DRIVE_KF = 0.0;

    public static final double ANGLE_P = 30;
    public static final double ANGLE_I = 0.0;
    public static final double ANGLE_D = 0.0;
    public static final double ANGLE_IZ = 0.0;

    //feedforward
    public static final double DRIVE_KS = 0;
    public static final double DRIVE_KV = 0;
    public static final double DRIVE_KA = 0;

    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    public static final Constraints DRIVE_CONSTRAINTS = new Constraints(
            MetersPerSecond.of(1000).in(MetersPerSecond),
            MetersPerSecondPerSecond.of(1000).in(MetersPerSecondPerSecond));

    public static final Constraints ANGLE_CONSTRAINTS = new Constraints(
            RadiansPerSecond.of(1000).in(RadiansPerSecond),
            RadiansPerSecondPerSecond.of(1000).in(RadiansPerSecondPerSecond));

    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double STEER_GEAR_RATIO = 150.0 / 7.0;
    public static final double WHEEL_RADIUS_METERS = 0.051;
    public static final Distance WHEEL_RADIUS = null;

    public static final double TRACK_WIDTH = 0.552;
    public static final double WHEEL_BASE = 0.552;
    public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS_METERS;

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
    );
    public static final double MAX_SPEED = 4.5;
    public static final double MAX_ANGULAR_VELOCITY = 10.0;

    public static final class FrontLeft {
        public static final int DRIVE_MOTOR_ID = 3;
        public static final int ANGLE_MOTOR_ID = 6;
        public static final int CANCODER_ID = 11;
        public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);
        public static final boolean isInverted = false;
        
    }

    public static final class FrontRight {
        public static final int DRIVE_MOTOR_ID = 1;
        public static final int ANGLE_MOTOR_ID = 7;
        public static final int CANCODER_ID = 9;
        public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);
        public static final boolean isInverted = false;
    }

    public static final class BackLeft {
        public static final int DRIVE_MOTOR_ID = 2;
        public static final int ANGLE_MOTOR_ID = 4;
        public static final int CANCODER_ID = 10;
        public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);
        public static final boolean isInverted = false;
    }

    public static final class BackRight {
        public static final int DRIVE_MOTOR_ID = 0;
        public static final int ANGLE_MOTOR_ID = 5;
        public static final int CANCODER_ID = 8;
        public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);
        public static final boolean isInverted = false;
    }
}
