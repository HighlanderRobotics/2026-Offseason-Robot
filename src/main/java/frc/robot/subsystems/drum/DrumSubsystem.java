package frc.robot.subsystems.drum;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DrumSubsystem extends SubsystemBase {
  public static final int FLYWHEEL_LEADER_ID = 9; // TODO: CORRECT ID
  // Ratio to main drum
  public static final double FLYWHEEL_GEAR_RATIO = 18 / 24;
  // May have to adjust 10/20 to account for the hood not moving a whole rotation
  public static final double HOOD_GEAR_RATIO = (12 / 30) * (10 / 20);

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

  public DrumSubsystem(CANBus canBus) {
    if (Robot.ROBOT_MODE != RobotMode.SIM) {
      flywheelIO = new FlywheelIO(canBus);

      hoodIO = new HoodIO(canBus);

      // TODO: CORRECT VALUES
      followerIOs[0] =
          new FollowerIO(
              0, FLYWHEEL_LEADER_ID, MotorAlignmentValue.Aligned, canBus, getFlywheelConfig());
      followerIOs[1] =
          new FollowerIO(
              0, FLYWHEEL_LEADER_ID, MotorAlignmentValue.Opposed, canBus, getFlywheelConfig());
      followerIOs[2] =
          new FollowerIO(
              0, FLYWHEEL_LEADER_ID, MotorAlignmentValue.Opposed, canBus, getFlywheelConfig());
    } else {
      flywheelIO = new FlywheelIOSim(canBus);
      hoodIO = new HoodIOSim(canBus);

      // TODO: CORRECT VALUES
      followerIOs[0] =
          new FollowerIOSim(
              10,
              FLYWHEEL_LEADER_ID,
              MotorAlignmentValue.Aligned,
              canBus,
              getFlywheelConfig(),
              () -> flywheelIOInputs.flywheelPositionRotations,
              () -> flywheelIOInputs.velocityRotPerSec);
      followerIOs[1] =
          new FollowerIOSim(
              11,
              FLYWHEEL_LEADER_ID,
              MotorAlignmentValue.Opposed,
              canBus,
              getFlywheelConfig(),
              () -> flywheelIOInputs.flywheelPositionRotations,
              () -> flywheelIOInputs.velocityRotPerSec);
      followerIOs[2] =
          new FollowerIOSim(
              12,
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

  // TODO: MORE COMMANDS WHEN SUPERSTRUCTURE IS INTEGRATED

  // Configs

  public static TalonFXConfiguration getFlywheelConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // TODO: VALUE FROM CAD
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode =
        NeutralModeValue.Brake; // Its possible that we should actually coast on this mech but idk

    // TODO: BUDGET CURRENT
    config.CurrentLimits.StatorCurrentLimit = 45.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = false;

    config.Feedback.SensorToMechanismRatio = DrumSubsystem.FLYWHEEL_GEAR_RATIO;

    config.MotionMagic.MotionMagicAcceleration = 10.0; // TODO: CALCULATE ACTUAL VALUE

    // Slot 0 is motion magic velocity pidf
    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 0.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    return config;
  }

  public static TalonFXConfiguration getHoodConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // TODO: VALUE FROM CAD
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // TODO: BUDGET CURRENT
    config.CurrentLimits.StatorCurrentLimit = 10.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = false;

    config.Feedback.SensorToMechanismRatio =
        DrumSubsystem.HOOD_GEAR_RATIO; // TODO: MAYBE INCLUDE CANCODER

    // TODO: CALCULATE ACTUAL VALUE
    config.MotionMagic.MotionMagicCruiseVelocity = 1.0;
    config.MotionMagic.MotionMagicAcceleration = 10.0;

    // Slot 0 is motion magic position pidf
    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 0.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    return config;
  }
}
