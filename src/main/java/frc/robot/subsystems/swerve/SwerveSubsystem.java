package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Volts;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Robot;
import frc.robot.Robot.RobotMode;
import frc.robot.subsystems.swerve.constants.AlphaSwerveConstants;
import frc.robot.subsystems.swerve.constants.SwerveConstants;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.gyro.GyroIOReal;
import frc.robot.subsystems.swerve.gyro.GyroIOSim;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleIOReal;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.odometry.OdometryThreadIO;
import frc.robot.subsystems.swerve.odometry.OdometryThreadIO.OdometryThreadIOInputs;
import frc.robot.subsystems.swerve.odometry.PhoenixOdometryThread;
import frc.robot.subsystems.swerve.odometry.PhoenixOdometryThread.Samples;
import frc.robot.subsystems.swerve.odometry.PhoenixOdometryThread.SignalID;
import frc.robot.subsystems.swerve.odometry.PhoenixOdometryThread.SignalType;
import frc.robot.utils.Tracer;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {
  // decide which set of swerve constants to use based on robot edition
  // defaulting to comp is probably safer?
  public static final SwerveConstants SWERVE_CONSTANTS = new AlphaSwerveConstants();

  private final Module[] modules; // Front Left, Front Right, Back Left, Back Right
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final OdometryThreadIO odometryThread;
  private final OdometryThreadIOInputs odometryThreadInputs = new OdometryThreadIOInputs();
  private double lastOdometryUpdateTimestamp = 0.0;

  private SwerveDriveKinematics kinematics;

  private SwerveDrivePoseEstimator estimator;

  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Rotation2d rawGyroRotation = new Rotation2d();

  private static final SignalID GYRO_SIGNAL_ID =
      new SignalID(SignalType.GYRO, PhoenixOdometryThread.GYRO_MODULE_ID);
  private static final SignalID[] DRIVE_SIGNAL_IDS = {
    new SignalID(SignalType.DRIVE, 0),
    new SignalID(SignalType.DRIVE, 1),
    new SignalID(SignalType.DRIVE, 2),
    new SignalID(SignalType.DRIVE, 3)
  };
  private static final SignalID[] TURN_SIGNAL_IDS = {
    new SignalID(SignalType.TURN, 0),
    new SignalID(SignalType.TURN, 1),
    new SignalID(SignalType.TURN, 2),
    new SignalID(SignalType.TURN, 3)
  };
  private Alert usingSyncOdoAlert = new Alert("Using Sync Odometry", AlertType.kInfo);
  private Alert missingModuleData = new Alert("Missing Module Data", AlertType.kError);
  private Alert missingGyroData = new Alert("Missing Gyro Data", AlertType.kWarning);

  private final SysIdRoutine turnSysid;

  private final SwerveModuleSimulationConfig mk5nModuleConfig =
      new SwerveModuleSimulationConfig(
          DCMotor.getKrakenX60Foc(1),
          DCMotor.getKrakenX44Foc(1),
          SwerveSubsystem.SWERVE_CONSTANTS.getDriveGearRatio(),
          SwerveSubsystem.SWERVE_CONSTANTS.getTurnGearRatio(),
          Volts.of(0.1),
          Volts.of(0.2),
          Inch.of(2),
          KilogramSquareMeters.of(0.03),
          2.25);
  // ripped the drive friction voltage, steer friction voltage, and steer rotational inertia from
  // the mk4n- no clue if that's right lmao
  // Maple Sim Stuff
  // TODO: UPDATE DRIVETRAIN SIM!!!
  private final DriveTrainSimulationConfig driveTrainSimConfig =
      DriveTrainSimulationConfig.Default()
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              // COTS.ofMark4n(
              //     DCMotor.getKrakenX60Foc(1),
              //     DCMotor.getKrakenX60Foc(1),
              //     // Still not sure where the 1.5 came from
              //     1.5,
              //     // Running l2+ swerve modules
              //     2))
              () -> new SwerveModuleSimulation(mk5nModuleConfig))
          .withTrackLengthTrackWidth(
              Meter.of(SwerveSubsystem.SWERVE_CONSTANTS.getTrackWidthX()),
              Meter.of(SwerveSubsystem.SWERVE_CONSTANTS.getTrackWidthY()))
          .withBumperSize(
              Meter.of(SwerveSubsystem.SWERVE_CONSTANTS.getBumperWidth()),
              Meter.of(SwerveSubsystem.SWERVE_CONSTANTS.getBumperLength()))
          .withRobotMass(SwerveSubsystem.SWERVE_CONSTANTS.getMass());

  private final SwerveDriveSimulation swerveSimulation =
      new SwerveDriveSimulation(driveTrainSimConfig, new Pose2d(3, 3, Rotation2d.kZero));

  public SwerveSubsystem(CANBus canbus) {
    if (Robot.ROBOT_MODE == RobotMode.SIM) {
      // Add simulated modules
      modules =
          new Module[] {
            new Module(
                new ModuleIOSim(
                    SWERVE_CONSTANTS.getFrontLeftModuleConstants(),
                    swerveSimulation.getModules()[0],
                    canbus)),
            new Module(
                new ModuleIOSim(
                    SWERVE_CONSTANTS.getFrontRightModuleConstants(),
                    swerveSimulation.getModules()[1],
                    canbus)),
            new Module(
                new ModuleIOSim(
                    SWERVE_CONSTANTS.getBackLeftModuleConstants(),
                    swerveSimulation.getModules()[2],
                    canbus)),
            new Module(
                new ModuleIOSim(
                    SWERVE_CONSTANTS.getBackRightModuleConstants(),
                    swerveSimulation.getModules()[3],
                    canbus))
          };
    } else {
      // Add real modules
      modules =
          new Module[] {
            new Module(new ModuleIOReal(SWERVE_CONSTANTS.getFrontLeftModuleConstants(), canbus)),
            new Module(new ModuleIOReal(SWERVE_CONSTANTS.getFrontRightModuleConstants(), canbus)),
            new Module(new ModuleIOReal(SWERVE_CONSTANTS.getBackLeftModuleConstants(), canbus)),
            new Module(new ModuleIOReal(SWERVE_CONSTANTS.getBackRightModuleConstants(), canbus))
          };
    }

    this.gyroIO =
        Robot.ROBOT_MODE != RobotMode.SIM
            ? new GyroIOReal(SWERVE_CONSTANTS.getGyroID(), SWERVE_CONSTANTS.getGyroConfig(), canbus)
            : new GyroIOSim(swerveSimulation.getGyroSimulation());

    this.kinematics = new SwerveDriveKinematics(SWERVE_CONSTANTS.getModuleTranslations());
    // Std devs copied from reefscape
    this.estimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            rawGyroRotation,
            lastModulePositions,
            new Pose2d(),
            VecBuilder.fill(0.6, 0.6, 0.07),
            VecBuilder.fill(0.9, 0.9, 0.4));

    this.odometryThread = PhoenixOdometryThread.getInstance();

    if (Robot.ROBOT_MODE == RobotMode.SIM) {
      SimulatedArena.getInstance().addDriveTrainSimulation(swerveSimulation);
    }

    this.turnSysid =
        new SysIdRoutine(
            new Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput(
                        "Swerve/" + modules[0].getPrefix() + "/Sysid State", state.toString())),
            new Mechanism((voltage) -> modules[0].setTurnVoltage(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    Tracer.trace(
        "Swerve Periodic",
        () -> {
          Tracer.trace(
              "Update odo thread inputs",
              () -> odometryThread.updateInputs(odometryThreadInputs, lastOdometryUpdateTimestamp));
          Logger.processInputs("AsyncOdo", odometryThreadInputs);
          if (!odometryThreadInputs.sampledStates.isEmpty()) {
            lastOdometryUpdateTimestamp =
                odometryThreadInputs
                    .sampledStates
                    .get(odometryThreadInputs.sampledStates.size() - 1)
                    .timestamp();
          }

          Tracer.trace("Update gyro inputs", () -> gyroIO.updateInputs(gyroInputs));
          Logger.processInputs("Swerve/Gyro", gyroInputs);

          for (Module module : modules) {
            Tracer.trace("Update module inputs for " + module.getPrefix(), module::periodic);
          }

          Tracer.trace("Update odometry", this::updateOdometry);

          // Logger.recordOutput("Current Hub Pose", FieldUtils.getCurrentHubPose());
        });
  }

  private void updateOdometry() {

    List<Samples> sampledStates = odometryThreadInputs.sampledStates;
    // Use sync samples if there aren't any async ones
    if (sampledStates.size() == 0
        || Robot.isSimulation()
        || sampledStates.get(0).values().isEmpty()) {
      usingSyncOdoAlert.set(true);
      sampledStates = getSyncSamples();
    } else {
      usingSyncOdoAlert.set(false);
    }
    // Update for each set of samples
    for (Samples sample : sampledStates) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      boolean hasNullModulePosition = false;
      boolean hasNullGyroRotation = false;
      // Get the positions and deltas for each module
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        Double dist = sample.values().get(DRIVE_SIGNAL_IDS[moduleIndex]);
        if (dist == null) {
          // No value at timestamp
          hasNullModulePosition = true;
          break;
        }

        Double rot = sample.values().get(TURN_SIGNAL_IDS[moduleIndex]);
        if (rot == null) {
          hasNullModulePosition = true;
          break;
        }

        // All data exists at this timestamp
        modulePositions[moduleIndex] =
            new SwerveModulePosition(dist, Rotation2d.fromRotations(rot)); // Values from thread
        // Change since last sample
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle.minus(lastModulePositions[moduleIndex].angle));

        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      hasNullGyroRotation =
          !gyroInputs.isConnected || (sample.values().get(GYRO_SIGNAL_ID) == null);

      // Set DS alerts
      missingModuleData.set(hasNullModulePosition);
      missingGyroData.set(hasNullGyroRotation);

      if (hasNullModulePosition && hasNullGyroRotation) {
        // Can't really do anything else rn bc theres no data
        continue;
      } else if (hasNullModulePosition && !hasNullGyroRotation) {
        rawGyroRotation = Rotation2d.fromDegrees(sample.values().get(GYRO_SIGNAL_ID));

        // If we're missing data, just update with the gyro and the previous module positions
        estimator.updateWithTime(sample.timestamp(), rawGyroRotation, lastModulePositions);
        continue;
      } else if (!hasNullModulePosition && hasNullGyroRotation) {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        // If theres no gyro data, update the rotation with the change in position
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      } else if (!hasNullModulePosition && !hasNullGyroRotation) {
        // We have all of our data
        rawGyroRotation = Rotation2d.fromDegrees(sample.values().get(GYRO_SIGNAL_ID));
      }

      // Apply update
      estimator.updateWithTime(sample.timestamp(), rawGyroRotation, modulePositions);
    }
  }

  /**
   * Runs the modules to the specified ChassisSpeeds (robot velocity)
   *
   * @param speeds the ChassisSpeeds to run the drivetrain at
   * @param openLoop boolean for if the drivetrain should run with feedforward control (open loop)
   *     or with feedback control (closed loop)
   */
  private void drive(ChassisSpeeds speeds, boolean openLoop) {
    // Converts time continuous chassis speeds to setpoints after the specified time (dtSeconds)
    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    // Convert drivetrain setpoint into individual module setpoints
    final SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    // Makes sure each wheel isn't asked to go above its max. Recalcs the states if needed
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SWERVE_CONSTANTS.getMaxLinearSpeed());
    Logger.recordOutput("SwerveStates/Setpoints", states);

    Logger.recordOutput("Swerve/Target Speeds", speeds);

    SwerveModuleState[] optimizedStates = new SwerveModuleState[modules.length];

    for (int i = 0; i < optimizedStates.length; i++) {
      if (openLoop) {
        // Heuristic to enable/disable FOC
        // enables FOC if the robot is moving at less than 90% of drivetrain max speed
        final boolean focEnable =
            Math.sqrt(
                    Math.pow(this.getVelocityRobotRelative().vxMetersPerSecond, 2)
                        + Math.pow(this.getVelocityRobotRelative().vyMetersPerSecond, 2))
                < SWERVE_CONSTANTS.getMaxLinearSpeed() * 0.9; // 0.9 is 90% of drivetrain max speed
        optimizedStates[i] = modules[i].runOpenLoop(states[i], focEnable);
      } else {
        optimizedStates[i] = modules[i].runClosedLoop(states[i]);
      }
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
  }

  /**
   * Drive closed-loop at robot relative speeds
   *
   * @param speeds robot relative speed setpoint
   * @return a command driving to target speeds
   */
  public Command driveClosedLoopRobotRelative(Supplier<ChassisSpeeds> speeds) {
    return this.run(() -> drive(speeds.get(), false)).withName("cr");
  }

  /**
   * Drive closed-loop at field relative speeds (i.e. for autoaim)
   *
   * @param speeds
   * @return a Command driving to the target speeds
   */
  public Command driveClosedLoopFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.run(
        () -> drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getRotation()), false));
  }

  public Command driveTrenchThing(Supplier<ChassisSpeeds> speeds) {
    return this.run(
            () -> {
              ChassisSpeeds speedRobotRelative =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds.get(),
                      // Flip so that speeds passed in are always relative to driver
                      DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                          ? getPose().getRotation()
                          : getPose().getRotation().minus(Rotation2d.fromDegrees(180)));
              this.drive(speedRobotRelative, false);
            })
        .withName("cf");
  }

  /**
   * Drive closed-loop at field relative speeds (i.e. for autoaim)
   *
   * @param speeds
   * @return a Command driving to the target speeds
   */
  public Command driveOpenLoopRobotRelative(Supplier<ChassisSpeeds> speeds) {
    return this.run(() -> drive(speeds.get(), true)).withName("or");
  }

  /**
   * Drives open-loop. Speeds field relative to driver. Used for teleop
   *
   * @param speeds the field-relative speeds to drive at
   * @return a Command driving at those speeds
   */
  public Command driveOpenLoopFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.run(
            () -> {
              ChassisSpeeds speedRobotRelative =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds.get(),
                      // Flip so that speeds passed in are always relative to driver
                      DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                          ? getPose().getRotation()
                          : getPose().getRotation().minus(Rotation2d.fromDegrees(180)));
              this.drive(speedRobotRelative, true);
            })
        .withName("of");
  }

  /**
   * Stops all the modules
   *
   * @return a command stopping all the modules
   */
  public Command stop() {
    // tbh i don't really know why this is a runonce?
    return this.runOnce(
        () -> {
          for (Module module : modules) {
            module.stop();
          }
        });
  }

  public Command stopForTime(DoubleSupplier seconds) {
    return stop().repeatedly().withTimeout(seconds.getAsDouble());
  }

  //   @AutoLogOutput(key = "Swerve/Near Trench")
  //   public boolean isNearTrench() {
  //     double x = getPose().getX();
  //     double y = getPose().getY();

  //     boolean inXTol =
  //         MathUtil.isNear(TrenchPoses.BLUE_RIGHT.getPose().getX(), x, 2)
  //             || MathUtil.isNear(TrenchPoses.RED_RIGHT.getPose().getX(), x, 2);
  //     boolean inYTol =
  //         MathUtil.isNear(TrenchPoses.BLUE_RIGHT.getPose().getY(), y, 0.515)
  //             || MathUtil.isNear(TrenchPoses.RED_RIGHT.getPose().getY(), y, 0.515);
  //     return inXTol && inYTol;
  //   }

  //   @AutoLogOutput(key = "Swerve/Near Trench for hood")
  //   public boolean isNearTrenchForHood() {
  //     double x = getPose().getX();
  //     double y = getPose().getY();

  //     boolean inXTol =
  //         MathUtil.isNear(TrenchPoses.BLUE_RIGHT.getPose().getX(), x, 1)
  //             || MathUtil.isNear(TrenchPoses.RED_RIGHT.getPose().getX(), x, 1);
  //     boolean inYTol =
  //         MathUtil.isNear(TrenchPoses.BLUE_RIGHT.getPose().getY(), y, 0.515)
  //             || MathUtil.isNear(TrenchPoses.RED_RIGHT.getPose().getY(), y, 0.515);
  //     return inXTol && inYTol;
  //   }

  //   public boolean isNearBump() {
  //     double x = getPose().getX();
  //     double y = getPose().getY();
  //     return (((Math.abs(x - FieldUtils.BLUE_BUMP_RIGHT_POS.getX()) < 2)
  //             || (Math.abs(x - FieldUtils.RED_BUMP_RIGHT_POS.getX()) < 2))
  //         && ((y > (FieldUtils.BLUE_BUMP_LEFT_POS.getY() - 0.515)
  //                 && y < (FieldUtils.BLUE_BUMP_LEFT_POS.getY() + 0.515)
  //             || (y > (FieldUtils.RED_BUMP_RIGHT_POS.getY() - 0.515)
  //                 && y < (FieldUtils.RED_BUMP_RIGHT_POS.getY() + 0.515)))));
  //   }

  /**
   * Generates a set of samples without using the async thread. Makes lots of Objects, so be careful
   * when using it irl!
   */
  private List<Samples> getSyncSamples() {
    return List.of(
        new Samples(
            Logger.getTimestamp() / 1.0e6,
            Map.of(
                new SignalID(SignalType.DRIVE, 0), modules[0].getPosition().distanceMeters,
                new SignalID(SignalType.TURN, 0), modules[0].getPosition().angle.getRotations(),
                new SignalID(SignalType.DRIVE, 1), modules[1].getPosition().distanceMeters,
                new SignalID(SignalType.TURN, 1), modules[1].getPosition().angle.getRotations(),
                new SignalID(SignalType.DRIVE, 2), modules[2].getPosition().distanceMeters,
                new SignalID(SignalType.TURN, 2), modules[2].getPosition().angle.getRotations(),
                new SignalID(SignalType.DRIVE, 3), modules[3].getPosition().distanceMeters,
                new SignalID(SignalType.TURN, 3), modules[3].getPosition().angle.getRotations(),
                new SignalID(SignalType.GYRO, PhoenixOdometryThread.GYRO_MODULE_ID),
                    gyroInputs.yaw.getDegrees())));
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return estimator.getEstimatedPosition();
  }

  public Pose3d getPose3d() {
    return new Pose3d(getPose());
  }

  /**
   * Returns the pose estimator rotation, as returned by {@link #getPose()}. between -180 and 180
   */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void resetPose(Pose2d newPose) {
    estimator.resetPose(newPose);
    if (Robot.ROBOT_MODE == RobotMode.SIM) {
      swerveSimulation.setSimulationWorldPose(newPose);
      swerveSimulation.setRobotSpeeds(new ChassisSpeeds());
    }
  }

  public void setYaw(Rotation2d newYaw) {
    resetPose(new Pose2d(getPose().getTranslation(), newYaw));
  }

  public void setGyroYaw(Rotation2d yaw) {
    gyroIO.setYaw(yaw);
  }

  @AutoLogOutput(key = "Odometry/Velocity Robot Relative")
  public ChassisSpeeds getVelocityRobotRelative() {
    ChassisSpeeds speeds = kinematics.toChassisSpeeds(getModuleStates());
    return speeds;
  }

  @AutoLogOutput(key = "Odometry/Velocity Field Relative")
  public ChassisSpeeds getVelocityFieldRelative() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getVelocityRobotRelative(), getRotation());
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states =
        Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
    return states;
  }

  /**
   * This function bypasses the command-based framework because Choreolib handles setting
   * requirements internally. Do NOT use outside of ChoreoLib
   *
   * @return a Consumer that runs the drivebase to follow a SwerveSample with PID feedback, sample
   *     target vel feedforward, and module force feedforward.
   */
  @SuppressWarnings("resource")
  public Consumer<SwerveSample> choreoDriveController() {
    // TODO: TUNE
    final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    final PIDController headingController = new PIDController(6.0, 0.0, 0.0);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    return (sample) -> {
      Pose2d pose = getPose();

      Logger.recordOutput("Choreo/Target Pose", sample.getPose());
      Logger.recordOutput("Choreo/Raw Target Speeds Field Relative", sample.getChassisSpeeds());

      ChassisSpeeds feedback =
          new ChassisSpeeds(
              xController.calculate(pose.getX(), sample.x),
              yController.calculate(pose.getY(), sample.y),
              headingController.calculate(pose.getRotation().getRadians(), sample.heading));

      ChassisSpeeds speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              sample.getChassisSpeeds().plus(feedback), getPose().getRotation());
      Logger.recordOutput("Choreo/Target Speeds Robot Relative", speeds);

      this.drive(speeds, false);
    };
  }

  public void resetMapleSimPose() {
    resetPose(swerveSimulation.getSimulatedDriveTrainPose());
  }

  @Override
  public void simulationPeriodic() {
    // Update maple simulation
    SimulatedArena.getInstance().simulationPeriodic();
    // Log simulated pose
    Logger.recordOutput("MapleSim/Pose", swerveSimulation.getSimulatedDriveTrainPose());
  }

  public Command runTurnSysid() {
    return Commands.sequence(
        turnSysid.quasistatic(Direction.kForward),
        turnSysid.quasistatic(Direction.kReverse),
        turnSysid.dynamic(Direction.kForward),
        turnSysid.dynamic(Direction.kReverse));
  }
}
