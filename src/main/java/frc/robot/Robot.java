package frc.robot;

import org.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystem.drivetrain.implementations.LimelightHelpers;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    //DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.drivetrainSimulation.setSimulationWorldPose(RobotConstants.INITIAL_POSE);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    m_robotContainer.autonomousPeriodic();
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
public void simulationPeriodic() {

    SimulatedArena.getInstance().simulationPeriodic();// burayı sil

    Pose2d simPose = m_robotContainer.drivetrainSimulation.getSimulatedDriveTrainPose();

    if (simPose != null) {
        double[] botPoseArr = new double[11];
        
        botPoseArr[0] = simPose.getX();
        botPoseArr[1] = simPose.getY();
        botPoseArr[2] = 0.0;
        botPoseArr[3] = 0.0;
        botPoseArr[4] = 0.0;
        botPoseArr[5] = simPose.getRotation().getDegrees();
        
        botPoseArr[6] = 0.0;
        botPoseArr[7] = 1.0;
        botPoseArr[8] = 0.0;
        botPoseArr[9] = 0.0;
        botPoseArr[10] = 0.0;

        LimelightHelpers.setLimelightNTDoubleArray("limelight", "botpose_wpiblue", botPoseArr);
        
        LimelightHelpers.setLimelightNTDouble("limelight", "tv", 1.0);
    }
}
}