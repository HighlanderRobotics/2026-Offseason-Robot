// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Robot;
import frc.robot.components.follower.FollowerIO;
import frc.robot.components.follower.FollowerIOInputsAutoLogged;
import frc.robot.components.follower.FollowerIOSim;

public class SlapdownSubsystem extends SubsystemBase {
  /** Creates a new SlapdownSubsystem. */
  public static final Rotation2d PIVOT_MIN_POSITION = Rotation2d.fromDegrees(0.0);
  public static final Rotation2d PIVOT_MAX_POSITION = Rotation2d.fromDegrees(130.0);
  public static final Rotation2d PIVOT_EXTENDED_POSITION = PIVOT_MIN_POSITION.plus(Rotation2d.fromDegrees(3.0));
  public static final Rotation2d PIVOT_RETRACTED_POSITION = PIVOT_MAX_POSITION;
  public static final double CURRENT_ZEROING_THRESHOLD = 20.0;
  public static final double ROLLER_GEAR_RATIO = 36.0 / 12.0;
  public static final double PIVOT_GEAR_RATIO = (60.0 / 8.0) * (64.0 / 16.0);
  public static final double PIVOT_TO_CANCODER = 1.0 / 1.77777778;
  public static final double CANCODER_TO_PIVOT = 1.77777778 / 1.0;

  private final PivotIO pivotIO;
  private PivotIOInputsAutoLogged = new PivotIOInputsAutoLogged();

  private final CANcoderIO cancoderIO;
  private CANcoderIOInputsAutoLogged = new CANcoderIOInputsAutoLogged();

  private final RollerIO rollerIO;
  private RollerIOInputsAutoLogged = new RollerIOInputsAutoLogged();

  private final FollowerIO followerIO; // i dident code the follower, so idk how it works
  private FollowerIOInputsAutoLogged = new FollowerIOInputsAutoLogged();

  private Trigger atExtensionTrigger = new Trigger(this::atExtension).debounce(0.01);

  private LinearFilter currentFilter = LinearFilter.movingAverage(5);

  private Alert pivotDisconnectAlert = new Alert("Intake pivot disconnected!", AlertType.kError);
  private Alert rollerLeaderDisconnected = new Alert("Intake leader disconnected!", AlertType.kError);
  private Alert rollerFollowerDisconnected = new Alert("Intake follower disconnected", AlertType.kError);
  private Alert cancoderDisconnected = new Alert("Intake cancoder disconnected!", AlertType.kError);

  @AutoLogOutput(key = "Intake/Pivot/Current Filter Value")
  private double currentFilterValue = 0.0;

  private SysIdRoutine rollerSysId;
  private SysIdRoutine pivotSysId;

  public SlapdownSubsystem(PivotIO pivotIO, CANcoderIO cancoderIO, RollerIO rollerIO, CANBus canBus) {
    // set this values to things in the constructor
    this.pivotIO = pivotIO;
    this.cancoderIO = cancoderIO;
    this.rollerIO = rollerIO;

    if (Robot.isSimulation()) {
      this.followerIO = new FollowerIOSim(10, rollerIO.getMotorId(), MotorAlignmentValue.Opposed, canBus, getRollerConfig(), () -> rollerIOInputs.positionRotaions, () -> rollerIOInputs.velocityRotsPerSecond);
    } else {
      this.followerIO = new FollowerIO(10, rollerIO.getMotorId(), MotorAlignmentValue.Opposed, canBus, getRollerConfig);
    }

    // sysids are copied and pasted
    rollerSysId =
      new SysIdRoutine(
        new Config(
          null,
          null,
          null,
          (state) -> Logger.recordOutput("Intake/Roller/SysID State", state.toString())),
        new Mechanism((voltage) -> rollerIO.setRollerVoltage(voltage.in(Volts)), null, this));
    pivotSysId =
      new SysIdRoutine(
        new Config(
          null,
          null,
          null,
          (state) -> Logger.recordOutput("Intake/Pivot/SysID State", state.toString())),
        new Mechanism((voltage) -> pivotIO.setMotorVoltage(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}