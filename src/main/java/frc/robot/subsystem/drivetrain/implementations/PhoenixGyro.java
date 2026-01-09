package frc.robot.subsystem.drivetrain.implementations;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystem.drivetrain.interfaces.IGyroInterface;

public class PhoenixGyro implements IGyroInterface {
    private final Pigeon2 pigeon;

    public PhoenixGyro(int deviceId) {
        this.pigeon = new Pigeon2(deviceId);
        pigeon.reset();
    }

    @Override
    public Rotation2d getAngle() {
        return pigeon.getRotation2d();
    }

    @Override
    public void setAngle(Rotation2d angle) {
        pigeon.setYaw(angle.getDegrees());
    }
}