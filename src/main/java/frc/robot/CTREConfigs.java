package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.constants.ModuleConstants;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import static edu.wpi.first.units.Units.Amps;

public final class CTREConfigs {

    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs() {
        // === CANCoder Config ===
        swerveCANcoderConfig.MagnetSensor.SensorDirection = ModuleConstants.cancoderInvert;

        // === Swerve Angle Motor Config ===
        swerveAngleFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        swerveAngleFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = ModuleConstants.STEER_GEAR_RATIO;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = ModuleConstants.ANGLE_ENABLE_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.ANGLE_CURRENT_LIMIT.in(Amps);

        swerveAngleFXConfig.Slot0.kP = ModuleConstants.ANGLE_P;
        swerveAngleFXConfig.Slot0.kI = ModuleConstants.ANGLE_I;
        swerveAngleFXConfig.Slot0.kD = ModuleConstants.ANGLE_D;

        // === Swerve Drive Motor Config ===
        swerveDriveFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        swerveDriveFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = ModuleConstants.DRIVE_GEAR_RATIO;

        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = ModuleConstants.DRIVE_ENABLE_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.DRIVE_CURRENT_LIMIT.in(Amps);

        swerveDriveFXConfig.Slot0.kP = ModuleConstants.DRIVE_P;
        swerveDriveFXConfig.Slot0.kI = ModuleConstants.DRIVE_I;
        swerveDriveFXConfig.Slot0.kD = ModuleConstants.DRIVE_D;

        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = ModuleConstants.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = ModuleConstants.OPEN_LOOP_RAMP;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = ModuleConstants.CLOSED_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ModuleConstants.CLOSED_LOOP_RAMP;
    }
}
