// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.CommandXboxControllerSubsystem;
import frc.robot.utils.EvergreenArena;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  /** Set to true to use logged tuneable numbers */
  public static final boolean TUNING_MODE = false;

  public enum RobotMode {
    REAL,
    SIM,
    REPLAY;
  }

  public static final RobotMode ROBOT_MODE = Robot.isReal() ? RobotMode.REAL : RobotMode.SIM;

  private CANBus canBus = new CANBus("*");

  private SwerveSubsystem swerve = new SwerveSubsystem(canBus);
  private IndexerSubsystem indexer = new IndexerSubsystem(canBus);

  private CommandXboxControllerSubsystem driver = new CommandXboxControllerSubsystem(0);
  private CommandXboxControllerSubsystem operator = new CommandXboxControllerSubsystem(1);

  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(false);
    SignalLogger.enableAutoLogging(false);
    RobotController.setBrownoutVoltage(6.0);

    // Metadata about the current code running on the robot
    Logger.recordMetadata("Codebase", "2026 Offseason");
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("Robot Mode", ROBOT_MODE.toString());
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    // log if we have uncommitted changes
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncommitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // set up logging stuff depending on robot mode
    switch (ROBOT_MODE) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        break;
    }
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.

    swerve.setDefaultCommand(
        swerve
            .driveOpenLoopFieldRelative(
                () ->
                    new ChassisSpeeds(
                            modifyJoystick(driver.getLeftY())
                                * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                            modifyJoystick(driver.getLeftX())
                                * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                            modifyJoystick(driver.getRightX())
                                * SwerveSubsystem.SWERVE_CONSTANTS.getMaxAngularSpeed())
                        .times(-1))
            .withName("Teleop drive"));

    indexer.setDefaultCommand(indexer.rest());

    driver.a().whileTrue(indexer.kick());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  // Use obstacle-free simulation arena
  static {
    SimulatedArena.overrideInstance(new EvergreenArena());
  }

  @Override
  public void simulationInit() {
    // Reset odo pose to maple sim pose
    swerve.resetMapleSimPose();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

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

  /** Scales a joystick value for teleop driving */
  private static double modifyJoystick(double val) {
    return MathUtil.applyDeadband(Math.abs(Math.pow(val, 2)) * Math.signum(val), 0.02);
  }
}
