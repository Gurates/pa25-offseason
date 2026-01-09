package frc.robot.subsystem.drivetrain.implementations;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystem.drivetrain.interfaces.IGyroInterface;

public class SimGyro implements IGyroInterface{

  private final GyroSimulation gyroSimulation;

  public SimGyro(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public Rotation2d getAngle() {
    return gyroSimulation.getGyroReading();
  }

  @Override
  public void setAngle(Rotation2d angle) {
    gyroSimulation.setRotation(angle);
  }
  
}