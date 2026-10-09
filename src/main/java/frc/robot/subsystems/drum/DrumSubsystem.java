package frc.robot.subsystems.drum;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Robot;
import frc.robot.Robot.RobotMode;
import frc.robot.components.follower.FollowerIO;
import frc.robot.components.follower.FollowerIOInputsAutoLogged;
import frc.robot.components.follower.FollowerIOSim;
import frc.robot.subsystems.drum.flywheel.FlywheelIO;
import frc.robot.subsystems.drum.flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.drum.flywheel.FlywheelIOSim;
import frc.robot.subsystems.drum.hood.HoodIO;
import frc.robot.subsystems.drum.hood.HoodIOInputsAutoLogged;
import frc.robot.subsystems.drum.hood.HoodIOSim;
import frc.robot.utils.autoaim.InterpolatingShotTree.ShotData;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DrumSubsystem extends SubsystemBase {
  public static final double HOOD_ANGLE_TOLERANCE_DEG = 2.0;
  public static final double FLYWHEEL_VEL_TOLERANCE_ROT_PER_SEC = 5.0;
  public static final int FLYWHEEL_LEADER_ID = 15;
  // Ratio to main drum
  public static final double FLYWHEEL_GEAR_RATIO = 24.0 / 18.0;
  // May have to adjust 10/20 to account for the hood not moving a whole rotation
  public static final double HOOD_GEAR_RATIO = (45.0 / 16.0) * (165.0 / 10.0);
  public static final Rotation2d HOOD_MIN_ANGLE = Rotation2d.fromDegrees(10);
  public static final Rotation2d HOOD_MAX_ANGLE = Rotation2d.fromDegrees(45);

  public static final double HOOD_CURRENT_ZEROING_THRESHOLD_AMPS = 30.0; // TODO: Tune

  private FlywheelIO flywheelIO;
  private FlywheelIOInputsAutoLogged flywheelIOInputs = new FlywheelIOInputsAutoLogged();

  private FollowerIO[] followerIOs = new FollowerIO[3];
  private FollowerIOInputsAutoLogged[] followerIOInputs = new FollowerIOInputsAutoLogged[3];

  private HoodIO hoodIO;
  private HoodIOInputsAutoLogged hoodIOInputs = new HoodIOInputsAutoLogged();

  private Alert hoodDisconnectAlert = new Alert("Hood Motor Disconnected", AlertType.kError);
  private Alert flywheelLeaderDisconnectAlert =
      new Alert("Flywheel Leader Disconnected", AlertType.kError);
  // True if any are disconnected (maybe I should add one for each but seems excessive)
  private Alert flywheelFollowerDisconnectAlert =
      new Alert("Flywheel Follower Disconnected", AlertType.kError);

  private SysIdRoutine flywheelSysid =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Drum/Flywheel/SysID State", state.toString())),
          new Mechanism((voltage) -> flywheelIO.setVoltage(voltage.in(Volts)), null, this));

  // TODO: PROBABLY NEED TO REDUCE RAMP RATE ETC TO MAKE WORK
  private SysIdRoutine hoodSysid =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Drum/Hood/SysID State", state.toString())),
          new Mechanism((voltage) -> hoodIO.setVoltage(voltage.in(Volts)), null, this));

  // For current zeroing
  private LinearFilter currentFilter = LinearFilter.movingAverage(10);
  private double currentFilterValue = 0.0;

  public DrumSubsystem(CANBus canBus) {
    if (Robot.ROBOT_MODE != RobotMode.SIM) {
      flywheelIO = new FlywheelIO(canBus);

      hoodIO = new HoodIO(canBus);

      followerIOs[0] =
          new FollowerIO(
              16, FLYWHEEL_LEADER_ID, MotorAlignmentValue.Aligned, canBus, getFlywheelConfig());
      followerIOs[1] =
          new FollowerIO(
              17, FLYWHEEL_LEADER_ID, MotorAlignmentValue.Opposed, canBus, getFlywheelConfig());
      followerIOs[2] =
          new FollowerIO(
              18, FLYWHEEL_LEADER_ID, MotorAlignmentValue.Opposed, canBus, getFlywheelConfig());
    } else {
      flywheelIO = new FlywheelIOSim(canBus);
      hoodIO = new HoodIOSim(canBus);

      followerIOs[0] =
          new FollowerIOSim(
              16,
              FLYWHEEL_LEADER_ID,
              MotorAlignmentValue.Aligned,
              canBus,
              getFlywheelConfig(),
              () -> flywheelIOInputs.flywheelPositionRotations,
              () -> flywheelIOInputs.velocityRotPerSec);
      followerIOs[1] =
          new FollowerIOSim(
              17,
              FLYWHEEL_LEADER_ID,
              MotorAlignmentValue.Opposed,
              canBus,
              getFlywheelConfig(),
              () -> flywheelIOInputs.flywheelPositionRotations,
              () -> flywheelIOInputs.velocityRotPerSec);
      followerIOs[2] =
          new FollowerIOSim(
              18,
              FLYWHEEL_LEADER_ID,
              MotorAlignmentValue.Opposed,
              canBus,
              getFlywheelConfig(),
              () -> flywheelIOInputs.flywheelPositionRotations,
              () -> flywheelIOInputs.velocityRotPerSec);
    }

    // Fill with blank inputs
    Arrays.fill(followerIOInputs, new FollowerIOInputsAutoLogged());
  }

  @Override
  public void periodic() {
    flywheelIO.updateInputs(flywheelIOInputs);
    Logger.processInputs("Drum/Flywheel/Leader", flywheelIOInputs);
    flywheelLeaderDisconnectAlert.set(!flywheelIOInputs.connected);

    // Update follower inputs
    boolean anyFollowerDisconnected = false;
    for (int i = 0; i < followerIOs.length; i++) {
      followerIOs[i].updateInputs(followerIOInputs[i]);
      Logger.processInputs("Drum/Flywheel/Follower " + i, followerIOInputs[i]);

      anyFollowerDisconnected |= !followerIOInputs[i].connected;
    }
    flywheelFollowerDisconnectAlert.set(anyFollowerDisconnected);

    hoodIO.updateInputs(hoodIOInputs);
    Logger.processInputs("Drum/Hood", hoodIOInputs);
    hoodDisconnectAlert.set(!hoodIOInputs.connected);
    currentFilterValue = currentFilter.calculate(hoodIOInputs.statorCurrentAmps);

    Logger.recordOutput("Drum/Hood/Setpoint", hoodIO.getAngleSetpoint());
    Logger.recordOutput("Drum/Flywheel/Setpoint", flywheelIO.getSetpointRotPerSec());

    if (Robot.isSimulation())
      Logger.recordOutput("Drum/Hood/Current Filter Value", currentFilterValue);
  }

  public Command setFlywheelAndHood(
      DoubleSupplier flywheelVelRotPerSec, Supplier<Rotation2d> hoodAngle) {
    return this.run(
        () -> {
          hoodIO.setPositionSetpoint(hoodAngle.get());
          flywheelIO.setVelocitySetpoint(flywheelVelRotPerSec.getAsDouble());
        });
  }

  public Command setFlywheelAndHoodVoltage(DoubleSupplier flywheel, DoubleSupplier hood) {
    return this.run(
        () -> {
          hoodIO.setVoltage(hood.getAsDouble());
          flywheelIO.setVoltage(flywheel.getAsDouble());
        });
  }

  public Command rest() {
    // Maybe should keep spinning somewhat
    return setFlywheelAndHood(() -> 0.0, () -> HOOD_MIN_ANGLE);
  }

  public Command shoot(Supplier<ShotData> shotDataSupplier) {
    return this.run(
        () -> {
          ShotData shotData = shotDataSupplier.get();
          hoodIO.setPositionSetpoint(shotData.hoodAngle());
          flywheelIO.setVelocitySetpoint(shotData.flywheelVelocityRotPerSec());
        });
  }

  public Command spit() {
    // Might need to tune this...
    return setFlywheelAndHood(() -> 30.0, () -> HOOD_MIN_ANGLE);
  }

  // TODO: MORE COMMANDS WHEN SUPERSTRUCTURE IS INTEGRATED

  // Current zeroing
  public Command runCurrentZeroing() {
    // TODO: May need to tune voltages etc.
    return this.run(() -> hoodIO.setVoltage(-3.0))
        .until(() -> currentFilterValue > HOOD_CURRENT_ZEROING_THRESHOLD_AMPS)
        .andThen(
            this.runOnce(hoodIO::rezeroMotorToBottom).alongWith(Commands.print("Rezeroed hood")));
  }

  // Sysids
  public Command runFlywheelSysid() {
    return Commands.sequence(
        flywheelSysid.quasistatic(Direction.kForward),
        Commands.waitUntil(
            () ->
                MathUtil.isNear(
                    0.0,
                    flywheelIOInputs.velocityRotPerSec,
                    1.0)), // Wait until we're nearly stopped
        flywheelSysid.quasistatic(Direction.kReverse),
        Commands.waitUntil(
            () ->
                MathUtil.isNear(
                    0.0,
                    flywheelIOInputs.velocityRotPerSec,
                    1.0)), // Wait until we're nearly stopped
        flywheelSysid.dynamic(Direction.kForward),
        Commands.waitUntil(
            () ->
                MathUtil.isNear(
                    0.0,
                    flywheelIOInputs.velocityRotPerSec,
                    1.0)), // Wait until we're nearly stopped
        flywheelSysid.dynamic(Direction.kReverse));
  }

  public Command runHoodSysid() {
    return Commands.sequence(
        hoodSysid
            .quasistatic(Direction.kForward)
            .until(() -> hoodIOInputs.position.getDegrees() > (HOOD_MAX_ANGLE.getDegrees() - 5)),
        hoodSysid
            .quasistatic(Direction.kReverse)
            .until(() -> hoodIOInputs.position.getDegrees() < (HOOD_MIN_ANGLE.getDegrees() + 5)),
        hoodSysid
            .dynamic(Direction.kForward)
            .until(() -> hoodIOInputs.position.getDegrees() > (HOOD_MAX_ANGLE.getDegrees() - 5)),
        hoodSysid
            .dynamic(Direction.kReverse)
            .until(() -> hoodIOInputs.position.getDegrees() < (HOOD_MIN_ANGLE.getDegrees() + 5)));
  }

  public boolean readyToShoot() {
    // TODO: TUNE tolerances
    return MathUtil.isNear(
            flywheelIO.getSetpointRotPerSec(),
            flywheelIOInputs.velocityRotPerSec,
            FLYWHEEL_VEL_TOLERANCE_ROT_PER_SEC)
        && MathUtil.isNear(
            hoodIO.getAngleSetpoint().getDegrees(),
            hoodIOInputs.position.getDegrees(),
            HOOD_ANGLE_TOLERANCE_DEG);
  }

  // Configs

  public static TalonFXConfiguration getFlywheelConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // TODO: VALUE FROM CAD
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // TODO: BUDGET CURRENT
    config.CurrentLimits.StatorCurrentLimit = 45.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = false;

    config.Feedback.SensorToMechanismRatio = DrumSubsystem.FLYWHEEL_GEAR_RATIO;

    config.MotionMagic.MotionMagicAcceleration = 10.0; // TODO: CALCULATE ACTUAL VALUE

    // Slot 0 is motion magic velocity pidf
    // TODO: RETUNE. FROM SIM
    config.Slot0.kS = 0.31;
    config.Slot0.kV = 0.17433;
    config.Slot0.kA = 0.10015;
    config.Slot0.kP = 1.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    return config;
  }

  public static TalonFXConfiguration getHoodConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // TODO: VALUE FROM CAD
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // TODO: BUDGET CURRENT
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = false;

    config.Feedback.SensorToMechanismRatio = DrumSubsystem.HOOD_GEAR_RATIO;

    // TODO: CALCULATE ACTUAL VALUE
    config.MotionMagic.MotionMagicCruiseVelocity = 1.0;
    config.MotionMagic.MotionMagicAcceleration = 10.0;

    // Slot 0 is motion magic position pidf
    config.Slot0.kS = 0.62;
    config.Slot0.kV = 5.0;
    config.Slot0.kG = 0.15;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 350.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 12.0;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    return config;
  }
}
