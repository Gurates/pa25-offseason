package frc.robot.subsystem.drivetrain.implementations;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.Conversions;
import frc.robot.constants.ModuleConstants;
import frc.robot.subsystem.drivetrain.interfaces.IModuleInterface;;

public class SimModule implements IModuleInterface {

  private final SwerveModuleSimulation moduleSimulation;

  private final SimulatedMotorController.GenericMotorController driveMotor;

  private final SimulatedMotorController.GenericMotorController angleMotor;

  private final ProfiledPIDController driveController, angleController;
  private TalonFX mDriveMotor; // fix

  public SimModule(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
    
    this.driveMotor = moduleSimulation
    .useGenericMotorControllerForDrive()
    .withCurrentLimit(ModuleConstants.DRIVE_CURRENT_LIMIT);

    this.angleMotor = moduleSimulation
    .useGenericControllerForSteer()
    .withCurrentLimit(ModuleConstants.ANGLE_CURRENT_LIMIT);

    this.driveController = new ProfiledPIDController(
      ModuleConstants.DRIVE_P, 
      ModuleConstants.DRIVE_I, 
      ModuleConstants.DRIVE_D, 
      ModuleConstants.DRIVE_CONSTRAINTS
    );
    
    driveController.setIZone(ModuleConstants.DRIVE_IZ);

    this.angleController = new ProfiledPIDController(
      ModuleConstants.ANGLE_P, 
      ModuleConstants.ANGLE_I, 
      ModuleConstants.ANGLE_D, 
      ModuleConstants.ANGLE_CONSTRAINTS
    );

    angleController.setIZone(ModuleConstants.ANGLE_IZ);

    angleController.enableContinuousInput(-Math.PI, +Math.PI);
  }

  @Override
  public void setDriveMotorVoltage(Voltage voltage) {
    driveMotor.requestVoltage(voltage);
  }

  @Override
  public void setAngleMotorVoltage(Voltage voltage) {
    angleMotor.requestVoltage(voltage);
  }

  @Override
  public void setModuleState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) <= 0.01) {
      desiredState.speedMetersPerSecond = 0;
    }

    desiredState.optimize(this.getModuleState().angle);

    Voltage driveVoltage = Volts.of(this.driveController.calculate(
      this.getModuleState().speedMetersPerSecond, 
      desiredState.speedMetersPerSecond
    ));

    Voltage angleVoltage = Volts.of(this.angleController.calculate(
      this.getModuleState().angle.getRadians(), 
      desiredState.angle.getRadians()
    ));

    setDriveMotorVoltage(driveVoltage);
    
    setAngleMotorVoltage(angleVoltage);
  }

  @Override
  public SwerveModuleState getModuleState() {
    return moduleSimulation.getCurrentState();
  }

  @Override
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
      moduleSimulation.getDriveWheelFinalPosition().in(Radians) * this.moduleSimulation.config.WHEEL_RADIUS.in(Meters),
      this.getModuleState().angle
    );
  }

  @Override
  public Angle getRawWheelPosition() {
    return moduleSimulation.getDriveWheelFinalPosition();
  }

  @Override
  public void stop() {
    driveMotor.requestVoltage(Volts.of(0));
    angleMotor.requestVoltage(Volts.of(0));
  }

  //fix
  @Override
    public double getDriveVelocity(){
        double nativeRps = mDriveMotor.getVelocity().getValueAsDouble();
        return Conversions.RPSToMPS(nativeRps, ModuleConstants.WHEEL_CIRCUMFERENCE);
    }
}