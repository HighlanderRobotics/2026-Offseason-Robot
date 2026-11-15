package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public class IndexerIO {

  public TalonFX indexer = new TalonFX(IndexerSubsystem.INDEXER_ROLLER_ID);
  public TalonFX kickerFollower = new TalonFX(IndexerSubsystem.KICKER_FOLLOWER_ID);
  public TalonFX kickerLeader = new TalonFX(IndexerSubsystem.KICKER_LEADER_ID);

  public VoltageOut indexerVoltageOut = new VoltageOut(0);
  public VoltageOut kickerLeaderVoltageOut = new VoltageOut(0);

  private final StatusSignal<Voltage> indexerAppliedVoltage = indexer.getMotorVoltage();
  private final StatusSignal<Voltage> kickerLeaderAppliedVoltage = kickerLeader.getMotorVoltage();
  private final StatusSignal<Voltage> kickerFollowerAppliedVoltage =
      kickerFollower.getMotorVoltage();

  private final StatusSignal<Angle> indexerPositionRots = indexer.getPosition();
  private final StatusSignal<Angle> kickerLeaderPositionRots = kickerLeader.getPosition();
  private final StatusSignal<Angle> kickerFollowerPositionRots = kickerFollower.getPosition();

  private final StatusSignal<Current> indexerSupplyCurrent = indexer.getSupplyCurrent();
  private final StatusSignal<Current> indexerStatorCurrent = indexer.getStatorCurrent();
  private final StatusSignal<Current> kickerLeaderSupplyCurrent = kickerLeader.getSupplyCurrent();
  private final StatusSignal<Current> kickerLeaderStatorCurrent = kickerLeader.getStatorCurrent();
  private final StatusSignal<Current> kickerFollowerSupplyCurrent =
      kickerFollower.getSupplyCurrent();
  private final StatusSignal<Current> kickerFollowerStatorCurrent =
      kickerFollower.getStatorCurrent();

  private final StatusSignal<AngularVelocity> indexerVelocity = indexer.getVelocity();
  private final StatusSignal<AngularVelocity> kickerLeaderVelocity = indexer.getVelocity();
  private final StatusSignal<AngularVelocity> kickerFollowerVelocity = indexer.getVelocity();

  private final StatusSignal<Temperature> indexerTemp = indexer.getDeviceTemp();
  private final StatusSignal<Temperature> kickerLeaderTemp = kickerLeader.getDeviceTemp();
  private final StatusSignal<Temperature> kickerFollowerTemp = kickerFollower.getDeviceTemp();

  public IndexerIO(CANBus canBus) {

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        indexerAppliedVoltage,
        kickerLeaderAppliedVoltage,
        kickerFollowerAppliedVoltage,
        indexerPositionRots,
        kickerLeaderPositionRots,
        kickerFollowerPositionRots,
        indexerSupplyCurrent,
        indexerStatorCurrent,
        kickerLeaderSupplyCurrent,
        kickerLeaderStatorCurrent,
        kickerFollowerSupplyCurrent,
        kickerFollowerStatorCurrent,
        indexerVelocity,
        kickerLeaderVelocity,
        kickerFollowerVelocity,
        indexerTemp,
        kickerLeaderTemp,
        kickerFollowerTemp);
    indexer.optimizeBusUtilization();
    kickerLeader.optimizeBusUtilization();
    kickerFollower.optimizeBusUtilization();

    // Set the follower to copy the leader
    kickerFollower.setControl(
        new Follower(IndexerSubsystem.KICKER_LEADER_ID, MotorAlignmentValue.Opposed));
  }

  @AutoLog
  public static class IndexerIOInputs {

    // required fields:
    // position, velocity, voltage, stator, supply, temp, positionRotations

    public double indexerVelocityRotsSec = 0f;
    public double indexerPositionRots = 0f;
    public double indexerStatorCurrent = 0f;
    public double indexerSupplyCurrent = 0f;
    public double indexerVoltage = 0.0;
    public double indexerTempC = 0.0;

    public double kickerLeaderVelocityRotsSec = 0f;
    public double kickerLeaderPositionRots = 0f;
    public double kickerLeaderStatorCurrent = 0f;
    public double kickerLeaderSupplyCurrent = 0f;
    public double kickerLeaderVoltage = 0f;
    public double kickerLeaderTempC = 0f;

    public double kickerFollowerVelocityRotsSec = 0f;
    public double kickerFollowerPositionRots = 0f;
    public double kickerFollowerStatorCurrent = 0f;
    public double kickerFollowerSupplyCurrent = 0f;
    public double kickerFollowerVoltage = 0f;
    public double kickerFollowerTempC = 0f;
  }

  public void updateInputs(IndexerIOInputs inputs) {

    BaseStatusSignal.refreshAll(
        indexerAppliedVoltage,
        kickerLeaderAppliedVoltage,
        kickerFollowerAppliedVoltage,
        indexerPositionRots,
        kickerLeaderPositionRots,
        kickerFollowerPositionRots,
        indexerSupplyCurrent,
        indexerStatorCurrent,
        kickerLeaderSupplyCurrent,
        kickerLeaderStatorCurrent,
        kickerFollowerSupplyCurrent,
        kickerFollowerStatorCurrent,
        indexerVelocity,
        kickerLeaderVelocity,
        kickerFollowerVelocity,
        indexerTemp,
        kickerLeaderTemp,
        kickerFollowerTemp);

    inputs.indexerVelocityRotsSec = indexerVelocity.getValueAsDouble();
    inputs.indexerPositionRots = indexerPositionRots.getValueAsDouble();
    inputs.indexerStatorCurrent = indexerStatorCurrent.getValueAsDouble();
    inputs.indexerSupplyCurrent = indexerSupplyCurrent.getValueAsDouble();
    inputs.indexerVoltage = indexerAppliedVoltage.getValueAsDouble();
    inputs.indexerTempC = indexerTemp.getValueAsDouble();

    inputs.kickerLeaderVelocityRotsSec = kickerLeaderVelocity.getValueAsDouble();
    inputs.kickerLeaderPositionRots = kickerLeaderPositionRots.getValueAsDouble();
    inputs.kickerLeaderStatorCurrent = kickerLeaderStatorCurrent.getValueAsDouble();
    inputs.kickerLeaderSupplyCurrent = kickerLeaderSupplyCurrent.getValueAsDouble();
    inputs.kickerLeaderVoltage = kickerLeaderAppliedVoltage.getValueAsDouble();
    inputs.kickerLeaderTempC = kickerLeaderTemp.getValueAsDouble();

    inputs.kickerFollowerVelocityRotsSec = kickerFollowerVelocity.getValueAsDouble();
    inputs.kickerFollowerPositionRots = kickerFollowerPositionRots.getValueAsDouble();
    inputs.kickerFollowerStatorCurrent = kickerFollowerStatorCurrent.getValueAsDouble();
    inputs.kickerFollowerSupplyCurrent = kickerFollowerSupplyCurrent.getValueAsDouble();
    inputs.kickerFollowerVoltage = kickerFollowerAppliedVoltage.getValueAsDouble();
    inputs.kickerFollowerTempC = kickerFollowerTemp.getValueAsDouble();
  }

  public void setIndexerVoltage(double volts) {
    indexer.setControl(indexerVoltageOut.withOutput(volts));
  }

  public void setKickerVoltage(double volts) {
    kickerLeader.setControl(kickerLeaderVoltageOut.withOutput(volts));
  }
}
