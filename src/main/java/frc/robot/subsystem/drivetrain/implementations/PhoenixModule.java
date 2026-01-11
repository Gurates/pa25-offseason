package frc.robot.subsystem.drivetrain.implementations;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.Units;

import frc.robot.constants.ModuleConstants;
import frc.robot.subsystem.drivetrain.interfaces.IModuleInterface;
import frc.robot.Robot;
import frc.robot.constants.Conversions;

public class PhoenixModule implements IModuleInterface {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(
            ModuleConstants.DRIVE_KS, ModuleConstants.DRIVE_KV, ModuleConstants.DRIVE_KA);

    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    private final int DRIVE_MOTOR_ID;
    private final int ANGLE_MOTOR_ID;
    private final int CANCODER_ID;

    public PhoenixModule(int moduleNumber) {
        this.moduleNumber = moduleNumber;

        switch (moduleNumber) {
            case 0:
                DRIVE_MOTOR_ID = ModuleConstants.FrontLeft.DRIVE_MOTOR_ID;
                ANGLE_MOTOR_ID = ModuleConstants.FrontLeft.ANGLE_MOTOR_ID;
                CANCODER_ID = ModuleConstants.FrontLeft.CANCODER_ID;
                angleOffset = ModuleConstants.FrontLeft.ANGLE_OFFSET;
                break;
            case 1:
                DRIVE_MOTOR_ID = ModuleConstants.FrontRight.DRIVE_MOTOR_ID;
                ANGLE_MOTOR_ID = ModuleConstants.FrontRight.ANGLE_MOTOR_ID;
                CANCODER_ID = ModuleConstants.FrontRight.CANCODER_ID;
                angleOffset = ModuleConstants.FrontRight.ANGLE_OFFSET;
                break;
            case 2:
                DRIVE_MOTOR_ID = ModuleConstants.BackLeft.DRIVE_MOTOR_ID;
                ANGLE_MOTOR_ID = ModuleConstants.BackLeft.ANGLE_MOTOR_ID;
                CANCODER_ID = ModuleConstants.BackLeft.CANCODER_ID;
                angleOffset = ModuleConstants.BackLeft.ANGLE_OFFSET;
                break;
            case 3:
                DRIVE_MOTOR_ID = ModuleConstants.BackRight.DRIVE_MOTOR_ID;
                ANGLE_MOTOR_ID = ModuleConstants.BackRight.ANGLE_MOTOR_ID;
                CANCODER_ID = ModuleConstants.BackRight.CANCODER_ID;
                angleOffset = ModuleConstants.BackRight.ANGLE_OFFSET;
                break;
            default:
                throw new IllegalArgumentException("Invalid moduleNumber: " + moduleNumber);
        }

        angleEncoder = new CANcoder(CANCODER_ID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        mAngleMotor = new TalonFX(ANGLE_MOTOR_ID);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        mDriveMotor = new TalonFX(DRIVE_MOTOR_ID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);


        // inverted motors (1-3 || 0-2)
        if(moduleNumber == 1 || moduleNumber == 3){
            MotorOutputConfigs driveConfigs = new MotorOutputConfigs();

            driveConfigs.Inverted = InvertedValue.Clockwise_Positive; // 
            mDriveMotor.getConfigurator().apply(driveConfigs);
        }
    }


    @Override
    public void setModuleState(SwerveModuleState desiredState) {
        desiredState = SwerveModuleState.optimize(desiredState, getModuleState().angle);

        anglePosition.withPosition(desiredState.angle.getRotations());
        mAngleMotor.setControl(anglePosition);

        setSpeed(desiredState, false);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / ModuleConstants.MAX_SPEED;
            mDriveMotor.setControl(driveDutyCycle);
        } else {
            double rps = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, ModuleConstants.WHEEL_CIRCUMFERENCE);
            driveVelocity.withVelocity(rps);
            driveVelocity.withFeedForward(driveFeedForward.calculate(desiredState.speedMetersPerSecond));
            mDriveMotor.setControl(driveVelocity);
        }
    }

    @Override
    public SwerveModuleState getModuleState() {
        double nativeRps = mDriveMotor.getVelocity().getValueAsDouble();
        double mps = Conversions.RPSToMPS(nativeRps, ModuleConstants.WHEEL_CIRCUMFERENCE);
        Rotation2d angle = Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble());
        return new SwerveModuleState(mps, angle);
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        double rotations = mDriveMotor.getPosition().getValueAsDouble();
        double meters = Conversions.rotationsToMeters(rotations, ModuleConstants.WHEEL_CIRCUMFERENCE);
        Rotation2d angle = Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble());
        return new SwerveModulePosition(meters, angle);
    }

    @Override
    public Angle getRawWheelPosition() {
        Rotation2d rot = getCANcoder();
        return Units.Radians.of(rot.getRadians());
    }

    @Override
    public void setDriveMotorVoltage(Voltage voltage) {
        double volts = voltage.in(Units.Volts);
        mDriveMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void setAngleMotorVoltage(Voltage voltage) {
        double volts = voltage.in(Units.Volts);
        mAngleMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void stop() {
        mDriveMotor.setControl(new VoltageOut(0.0));
        mAngleMotor.setControl(new VoltageOut(0.0));
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }
}
