package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystem.drivetrain.enums.DriveType;

public class DriveCommand extends Command {

    private final DrivetrainSubsystem drivetrainSubsystem;

    private final DriveType driveType;

    private final Supplier<Double> xSupplier, ySupplier, rSupplier;

    private double xInput, yInput, rInput;

    public DriveCommand(
            DrivetrainSubsystem drivetrainSubsystem,
            DriveType driveType,
            Supplier<Double> xSupplier,
            Supplier<Double> ySupplier,
            Supplier<Double> rSupplier) {
        this.drivetrainSubsystem = drivetrainSubsystem;

        this.driveType = driveType;

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rSupplier = rSupplier;

        this.addRequirements(this.drivetrainSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        xInput = xSupplier.get();
        yInput = ySupplier.get();

        rInput = rSupplier.get();

        drivetrainSubsystem.drive(yInput, xInput, rInput, driveType);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}