// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drum.DrumSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.CANcoderIO;
import frc.robot.subsystems.intake.PivotIO;
import frc.robot.subsystems.intake.PivotIOSim;
import frc.robot.subsystems.intake.RollerIO;
import frc.robot.subsystems.intake.RollerIOSim;
import frc.robot.subsystems.intake.SlapdownSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.CommandXboxControllerSubsystem;
import frc.robot.utils.EvergreenArena;
import java.util.Optional;
import java.util.Set;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
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
  private DrumSubsystem drum = new DrumSubsystem(canBus);
  private SlapdownSubsystem intake =
      Robot.isSimulation()
          ? new SlapdownSubsystem(
              new PivotIOSim(8, SlapdownSubsystem.getPivotConfig(), canBus),
              new CANcoderIO(4, SlapdownSubsystem.getCancoderConfig(), canBus),
              new RollerIOSim(9, SlapdownSubsystem.getRollerConfig(), canBus),
              canBus)
          : new SlapdownSubsystem(
              new PivotIO(8, SlapdownSubsystem.getPivotConfig(), canBus),
              new CANcoderIO(4, SlapdownSubsystem.getCancoderConfig(), canBus),
              new RollerIO(9, SlapdownSubsystem.getRollerConfig(), canBus),
              canBus);

  private CommandXboxControllerSubsystem driver = new CommandXboxControllerSubsystem(0);
  private CommandXboxControllerSubsystem operator = new CommandXboxControllerSubsystem(1);

  private Superstructure superstructure =
      new Superstructure(driver, operator, indexer, drum, intake, swerve::getPose);

  private LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto");
  private Optional<Alliance> lastAlliance = Optional.empty();

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

    SmartDashboard.putData("Add autos", Commands.runOnce(this::addAutos).ignoringDisable(true));

    // swerve.setDefaultCommand(
    //     swerve
    //         .driveOpenLoopFieldRelative(
    //             () ->
    //                 new ChassisSpeeds(
    //                         modifyJoystick(driver.getLeftY())
    //                             * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
    //                         modifyJoystick(driver.getLeftX())
    //                             * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
    //                         modifyJoystick(driver.getRightX())
    //                             * SwerveSubsystem.SWERVE_CONSTANTS.getMaxAngularSpeed())
    //                     .times(-1))
    //         .withName("Teleop drive"));
    swerve.setDefaultCommand(
        swerve.driveOpenLoopRobotRelative(
            () ->
                new ChassisSpeeds(
                        modifyJoystick(driver.getLeftY())
                            * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                        modifyJoystick(driver.getLeftX())
                            * SwerveSubsystem.SWERVE_CONSTANTS.getMaxLinearSpeed(),
                        modifyJoystick(driver.getRightX())
                            * SwerveSubsystem.SWERVE_CONSTANTS.getMaxAngularSpeed())
                    .times(-1)));

    indexer.setDefaultCommand(indexer.rest());
    intake.setDefaultCommand(intake.restExtended());
    drum.setDefaultCommand(drum.rest());

    drum.setDefaultCommand(drum.setFlywheelAndHoodVoltage(() -> 0.0, () -> 0.0));

    driver.a().whileTrue(drum.setFlywheelAndHoodVoltage(() -> 10.0, () -> 10.0));
    driver.b().whileTrue(drum.runCurrentZeroing());

    // Run auto when auto starts. Matches Choreolib's defer impl
    RobotModeTriggers.autonomous()
        .whileTrue(Commands.defer(() -> autoChooser.get().asProxy(), Set.of()));

    // Add autos on alliance change
    new Trigger(
            () -> {
              boolean allianceChanged = !DriverStation.getAlliance().equals(lastAlliance);
              lastAlliance = DriverStation.getAlliance();
              return allianceChanged && DriverStation.getAlliance().isPresent();
            })
        .onTrue(Commands.runOnce(() -> addAutos()));
  }

  private void addAutos() {
    autoChooser.addDefaultOption("None", Commands.none());

    // Sysids
    autoChooser.addOption("Hood Sysid", drum.runHoodSysid());
    autoChooser.addOption("Flywheel Sysid", drum.runFlywheelSysid());
    autoChooser.addOption("Indexer Sysid", indexer.runIndexerSysid());
    autoChooser.addOption("Kicker Sysid", indexer.runKickerSysid());
    autoChooser.addOption("Intake Roller Sysid", intake.runRollerSysid());
    autoChooser.addOption("Intake Pivot Sysid", intake.runPivotSysid());
    autoChooser.addDefaultOption("Swerve turn sysid", swerve.runTurnSysid());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void simulationPeriodic() {
    superstructure.simulationPeriodic();
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
