package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.susbsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.susbsystems.drivetrain.enums.DriveType;
import frc.robot.susbsystems.drivetrain.implementations.PhoenixGyro;
import frc.robot.susbsystems.drivetrain.implementations.PhoenixModule;
import frc.robot.susbsystems.drivetrain.implementations.SimGyro;
import frc.robot.susbsystems.drivetrain.implementations.SimModule;
import frc.robot.susbsystems.drivetrain.interfaces.IGyroInterface;
import frc.robot.susbsystems.drivetrain.interfaces.IModuleInterface;
import frc.robot.utils.Logger;

public class RobotContainer {

    CommandXboxController controller = new CommandXboxController(ControllerConstants.CONTROLLER_PORT);

    Trigger x = controller.x();
    Trigger a = controller.a();
    Trigger b = controller.b();
    Trigger y = controller.y();
    Trigger rb = controller.rightBumper();
    Trigger rt = controller.rightTrigger(0.1);
    Trigger lb = controller.leftBumper();
    Trigger lt = controller.leftTrigger(0.1);

    Trigger povUp = controller.povUp();
    Trigger povDown = controller.povDown();
    Trigger povLeft = controller.povLeft();
    Trigger povRight = controller.povRight();

    public final SwerveDriveSimulation drivetrainSimulation;
    public final DrivetrainSubsystem drivetrainSubsystem;

    DriveCommand fieldRelativeDriveCommand;
    DriveCommand robotRelativeDriveCommand;

    SendableChooser<Command> autoChooser;

    public RobotContainer() {
        
        IGyroInterface gyro;
        IModuleInterface fl, fr, bl, br;

        
        if (RobotBase.isReal()) {
            drivetrainSimulation = new SwerveDriveSimulation(
                DrivetrainConstants.DRIVETRAIN_CONFIG,
                RobotConstants.INITIAL_POSE
            );

            SimulatedArena.getInstance().addDriveTrainSimulation(drivetrainSimulation);
            SimulatedArena.getInstance().resetFieldForAuto();

            gyro = new SimGyro(drivetrainSimulation.getGyroSimulation());
            fl = new SimModule(drivetrainSimulation.getModules()[0]);
            fr = new SimModule(drivetrainSimulation.getModules()[1]);
            bl = new SimModule(drivetrainSimulation.getModules()[2]);
            br = new SimModule(drivetrainSimulation.getModules()[3]);

        } else {
            drivetrainSimulation = null;

            // gyro ıd
            gyro = new PhoenixGyro(12);

            fl = new PhoenixModule(0);
            fr = new PhoenixModule(1);
            bl = new PhoenixModule(2);
            br = new PhoenixModule(3);
        }

        drivetrainSubsystem = new DrivetrainSubsystem(gyro, fl, fr, bl, br);

        autoChooser = AutoBuilder.buildAutoChooser();

        this.robotRelativeDriveCommand = new DriveCommand(
            drivetrainSubsystem,
            DriveType.RobotRelative,
            () -> Math.abs(controller.getLeftX()) > ControllerConstants.DEADBAND ? controller.getLeftX() * -RobotConstants.MAX_SPEED.in(edu.wpi.first.units.Units.MetersPerSecond) : 0,
            () -> Math.abs(controller.getLeftY()) > ControllerConstants.DEADBAND ? controller.getLeftY() * -RobotConstants.MAX_SPEED.in(edu.wpi.first.units.Units.MetersPerSecond) : 0,
            () -> Math.abs(controller.getRightX()) > ControllerConstants.DEADBAND ? controller.getRightX() * -RobotConstants.MAX_SPEED.in(edu.wpi.first.units.Units.MetersPerSecond) : 0
        );

        this.fieldRelativeDriveCommand = new DriveCommand(
            drivetrainSubsystem,
            DriveType.FieldRelative,
            () -> Math.abs(controller.getLeftX()) > ControllerConstants.DEADBAND ? controller.getLeftX() * -RobotConstants.MAX_SPEED.in(edu.wpi.first.units.Units.MetersPerSecond) : 0,
            () -> Math.abs(controller.getLeftY()) > ControllerConstants.DEADBAND ? controller.getLeftY() * -RobotConstants.MAX_SPEED.in(edu.wpi.first.units.Units.MetersPerSecond) : 0,
            () -> Math.abs(controller.getRightX()) > ControllerConstants.DEADBAND ? controller.getRightX() * -RobotConstants.MAX_SPEED.in(edu.wpi.first.units.Units.MetersPerSecond) : 0
        );

        configureBindings();
    }

    public void configureBindings() {
        drivetrainSubsystem.setDefaultCommand(fieldRelativeDriveCommand);

        if (RobotBase.isSimulation()) {
            x.onTrue(Commands.runOnce(() -> {
                if(drivetrainSimulation != null) {
                    drivetrainSubsystem.setPose(drivetrainSimulation.getSimulatedDriveTrainPose());
                }
            }));
        } else {
            x.onTrue(Commands.runOnce(() -> drivetrainSubsystem.setPose(new edu.wpi.first.math.geometry.Pose2d())));
        }
    }

    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto(autoChooser.getSelected().getName());
    }

    public void autonomousPeriodic() {
        if (RobotBase.isSimulation() && drivetrainSimulation != null) {
            drivetrainSubsystem.setPose(drivetrainSimulation.getSimulatedDriveTrainPose());
        }
    }

    public void periodic() {
        if (AutoBuilder.isConfigured()) {
            Logger.log("AutoChooser/SelectedAuto", autoChooser);
        }
        
        if (RobotBase.isSimulation() && drivetrainSimulation != null) {
            Logger.log("Drivetrain/Poses/SimPose", drivetrainSimulation.getSimulatedDriveTrainPose());
        }
    }
}