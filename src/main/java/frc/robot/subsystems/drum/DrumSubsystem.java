package frc.robot.subsystems.drum;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.follower.FollowerIO;
import frc.robot.components.follower.FollowerIOInputsAutoLogged;
import frc.robot.subsystems.drum.flywheel.FlywheelIO;
import frc.robot.subsystems.drum.flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.drum.hood.HoodIO;
import frc.robot.subsystems.drum.hood.HoodIOInputsAutoLogged;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DrumSubsystem extends SubsystemBase {
  public static final int FLYWHEEL_LEADER_ID = 0; // TODO: CORRECT ID
  public static final double FLYWHEEL_GEAR_RATIO = 1.0; // TODO: VALUE FROM CAD
  public static final double HOOD_GEAR_RATIO = 1.0; // TODO: VALUE FROM CAD

  private FlywheelIO flywheelIO;
  private FlywheelIOInputsAutoLogged flywheelIOInputs = new FlywheelIOInputsAutoLogged();

  private FollowerIO[] followerIOs = new FollowerIO[3];
  private FollowerIOInputsAutoLogged[] followerIOInputs = new FollowerIOInputsAutoLogged[3];

  private HoodIO hoodIO;
  private HoodIOInputsAutoLogged hoodIOInputs = new HoodIOInputsAutoLogged();

  private Alert hoodDisconnectAlert = new Alert("Hood Motor Disconnected", AlertType.kError);
  private Alert flywheelLeaderDisconnectAlert = new Alert("Flywheel Leader Disconnected", AlertType.kError);
  // True if any are disconnected (maybe I should add one for each but seems excessive)
  private Alert flywheelFollowerDisconnectAlert = new Alert("Flywheel Follower Disconnected", AlertType.kError);

  public DrumSubsystem(CANBus canBus) {
    flywheelIO = new FlywheelIO(canBus);

    // Fill with blank inputs
    Arrays.fill(followerIOInputs, new FollowerIOInputsAutoLogged());

    // TODO: CORRECT VALUES
    followerIOs[0] = new FollowerIO(0, FLYWHEEL_LEADER_ID, null, canBus, getFlywheelConfig());
    followerIOs[1] = new FollowerIO(0, FLYWHEEL_LEADER_ID, null, canBus, getFlywheelConfig());
    followerIOs[2] = new FollowerIO(0, FLYWHEEL_LEADER_ID, null, canBus, getFlywheelConfig());
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
