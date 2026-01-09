package frc.robot.subsystem.drivetrain.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IGyroInterface {
    public Rotation2d getAngle();
    public void setAngle(Rotation2d angle);
} 