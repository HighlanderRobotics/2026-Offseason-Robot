package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public class IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double indexerVelocityRotationsPerSec = 0.0;
    public double indexerPositionRots = 0.0;
    public double indexerStatorCurrentAmp = 0.0;
    public double indexerSupplyCurrentAmp = 0.0;
    public double indexerVoltage = 0.0;
    public double indexerTempC = 0.0;
    public boolean indexerConnected = false;

    public double kickerLeaderVelocityRotationsPerSec = 0.0;
    public double kickerLeaderPositionRots = 0.0;
    public double kickerLeaderStatorCurrentAmp = 0.0;
    public double kickerLeaderSupplyCurrentAmp = 0.0;
    public double kickerLeaderVoltage = 0.0;
    public double kickerLeaderTempC = 0.0;
    public boolean kickerLeaderConnected = false;

    public double kickerFollowerVelocityRotationsPerSec = 0.0;
    public double kickerFollowerPositionRots = 0.0;
    public double kickerFollowerStatorCurrentAmp = 0.0;
    public double kickerFollowerSupplyCurrentAmp = 0.0;
    public double kickerFollowerVoltage = 0.0;
    public double kickerFollowerTempC = 0.0;
    public boolean kickerFollowerConnected = false;
  }

  // Set gear ratios
  public static final double GEAR_RATIO = 50/12;
  public static final double KICKER_GEAR_RATIO = 30/12;

  protected final TalonFX indexerMotor;
  protected final TalonFX kickerLeaderMotor;
  protected final TalonFX kickerFollowerMotor;

  // StatusSignal - used to get information about each motor
  private final StatusSignal<AngularVelocity> indexerAngularVelocityRotsPerSec;
  private final StatusSignal<Angle> indexerPosition;
  private final StatusSignal<Current> indexerStatorCurrent;
  private final StatusSignal<Current> indexerSupplyCurrent;
  private final StatusSignal<Voltage> indexerVoltage;
  private final StatusSignal<Temperature> indexerTemp;

  private final StatusSignal<AngularVelocity> kickerLeaderAngularVelocityRotsPerSec;
  private final StatusSignal<Angle> kickerLeaderPosition;
  private final StatusSignal<Current> kickerLeaderStatorCurrent;
  private final StatusSignal<Current> kickerLeaderSupplyCurrent;
  private final StatusSignal<Voltage> kickerLeaderVoltage;
  private final StatusSignal<Temperature> kickerLeaderTemp;

  private final StatusSignal<AngularVelocity> kickerFollowerAngularVelocityRotsPerSec;
  private final StatusSignal<Angle> kickerFollowerPosition;
  private final StatusSignal<Current> kickerFollowerStatorCurrent;
  private final StatusSignal<Current> kickerFollowerSupplyCurrent;
  private final StatusSignal<Voltage> kickerFollowerVoltage;
  private final StatusSignal<Temperature> kickerFollowerTemp;

  // Voltage and velocity controllers
  private VoltageOut voltageOut =
      new VoltageOut(0.0).withEnableFOC((true)); // FOC - higher power and smoother
  private VelocityVoltage velocityVoltage =
      new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);

  public IndexerIO(CANBus canBus) {
    // TODO: set motor ID for indexer
    indexerMotor = new TalonFX(16, canBus);
    indexerMotor.getConfigurator().apply(IndexerIO.getIndexerConfiguration());

    // TODO: set motor ID for kicker leader
    kickerLeaderMotor = new TalonFX(15, canBus);
    kickerLeaderMotor.getConfigurator().apply(IndexerIO.getKickerConfiguration());

    // TODO: set motor ID for kicker follower
    kickerFollowerMotor = new TalonFX(14, canBus);
    kickerFollowerMotor.getConfigurator().apply(IndexerIO.getKickerConfiguration());

    // Set kicker follower to follow leader
    kickerFollowerMotor.setControl(new Follower(kickerLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    // Set the data for each motor
    indexerAngularVelocityRotsPerSec = indexerMotor.getVelocity();
    indexerPosition = indexerMotor.getPosition();
    indexerStatorCurrent = indexerMotor.getStatorCurrent();
    indexerSupplyCurrent = indexerMotor.getSupplyCurrent();
    indexerVoltage = indexerMotor.getMotorVoltage();
    indexerTemp = indexerMotor.getDeviceTemp();

    kickerLeaderAngularVelocityRotsPerSec = kickerLeaderMotor.getVelocity();
    kickerLeaderPosition = kickerLeaderMotor.getPosition();
    kickerLeaderStatorCurrent = kickerLeaderMotor.getStatorCurrent();
    kickerLeaderSupplyCurrent = kickerLeaderMotor.getSupplyCurrent();
    kickerLeaderVoltage = kickerLeaderMotor.getMotorVoltage();
    kickerLeaderTemp = kickerLeaderMotor.getDeviceTemp();

    kickerFollowerAngularVelocityRotsPerSec = kickerFollowerMotor.getVelocity();
    kickerFollowerPosition = kickerFollowerMotor.getPosition();
    kickerFollowerStatorCurrent = kickerFollowerMotor.getStatorCurrent();
    kickerFollowerSupplyCurrent = kickerFollowerMotor.getSupplyCurrent();
    kickerFollowerVoltage = kickerFollowerMotor.getMotorVoltage();
    kickerFollowerTemp = kickerFollowerMotor.getDeviceTemp();

    // Use setUpdateFrequencyForAll to only update motor data every time the robot updates
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        indexerAngularVelocityRotsPerSec,
        indexerPosition,
        indexerStatorCurrent,
        indexerSupplyCurrent,
        indexerStatorCurrent,
        indexerVoltage,
        indexerTemp,
        kickerLeaderAngularVelocityRotsPerSec,
        kickerLeaderPosition,
        kickerLeaderStatorCurrent,
        kickerLeaderSupplyCurrent,
        kickerLeaderStatorCurrent,
        kickerLeaderVoltage,
        kickerLeaderTemp,
        kickerFollowerAngularVelocityRotsPerSec,
        kickerFollowerPosition,
        kickerFollowerStatorCurrent,
        kickerFollowerSupplyCurrent,
        kickerFollowerStatorCurrent,
        kickerFollowerVoltage,
        kickerFollowerTemp);
    indexerMotor
        .optimizeBusUtilization(); // only update variables that have update frequency set to non-zero value
    kickerLeaderMotor.optimizeBusUtilization();
    kickerFollowerMotor.optimizeBusUtilization();
  }

  public static TalonFXConfiguration getIndexerConfiguration() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // precise stopping
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // sets direction

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    // Set PID values
    config.Slot0.kS = 0;
    config.Slot0.kG = 0;
    config.Slot0.kV = 0;
    config.Slot0.kP = 0;
    config.Slot0.kD = 0;

    // Limits for current
    config.CurrentLimits.StatorCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    return config;
  }

  public static TalonFXConfiguration getKickerConfiguration() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // precise stopping
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // sets direction

    config.Feedback.SensorToMechanismRatio = KICKER_GEAR_RATIO;

    // Set PID values
    config.Slot0.kS = 0;
    config.Slot0.kG = 0;
    config.Slot0.kV = 0;
    config.Slot0.kP = 0;
    config.Slot0.kD = 0;

    // Limits for current
    config.CurrentLimits.StatorCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    return config;
  }

  public void setIndexerVoltage(double voltage) {
    indexerMotor.setControl(voltageOut.withOutput(voltage));
  }

  public void setKickerVoltage(double voltage) {
    kickerLeaderMotor.setControl(voltageOut.withOutput(voltage));
  }

  public void updateInputs(IndexerIOInputs inputs) {
    // Refresh all variables
    BaseStatusSignal.refreshAll(
        indexerAngularVelocityRotsPerSec,
        indexerPosition,
        indexerStatorCurrent,
        indexerSupplyCurrent,
        indexerVoltage,
        indexerTemp,
        kickerLeaderAngularVelocityRotsPerSec,
        kickerLeaderPosition,
        kickerLeaderStatorCurrent,
        kickerLeaderSupplyCurrent,
        kickerLeaderVoltage,
        kickerLeaderTemp,
        kickerFollowerAngularVelocityRotsPerSec,
        kickerFollowerPosition,
        kickerFollowerStatorCurrent,
        kickerFollowerSupplyCurrent,
        kickerFollowerVoltage,
        kickerFollowerTemp);

    // Set variables to the motor data for indexer
    inputs.indexerConnected =
        BaseStatusSignal.isAllGood(
            indexerAngularVelocityRotsPerSec,
            indexerPosition,
            indexerStatorCurrent,
            indexerSupplyCurrent,
            indexerVoltage,
            indexerTemp);
    inputs.indexerVelocityRotationsPerSec = indexerAngularVelocityRotsPerSec.getValueAsDouble();
    inputs.indexerPositionRots = indexerPosition.getValueAsDouble();
    inputs.indexerStatorCurrentAmp = indexerStatorCurrent.getValueAsDouble();
    inputs.indexerSupplyCurrentAmp = indexerSupplyCurrent.getValueAsDouble();
    inputs.indexerVoltage = indexerVoltage.getValueAsDouble();
    inputs.indexerTempC = indexerTemp.getValueAsDouble();

    // Set variables to the motor data for kicker leader
    inputs.kickerLeaderConnected =
        BaseStatusSignal.isAllGood(
            kickerLeaderAngularVelocityRotsPerSec,
            kickerLeaderPosition,
            kickerLeaderStatorCurrent,
            kickerLeaderSupplyCurrent,
            kickerLeaderVoltage,
            kickerLeaderTemp);
    inputs.kickerLeaderVelocityRotationsPerSec = kickerLeaderAngularVelocityRotsPerSec.getValueAsDouble();
    inputs.kickerLeaderPositionRots = kickerLeaderPosition.getValueAsDouble();
    inputs.kickerLeaderStatorCurrentAmp = kickerLeaderStatorCurrent.getValueAsDouble();
    inputs.kickerLeaderSupplyCurrentAmp = kickerLeaderSupplyCurrent.getValueAsDouble();
    inputs.kickerLeaderVoltage = kickerLeaderVoltage.getValueAsDouble();
    inputs.kickerLeaderTempC = kickerLeaderTemp.getValueAsDouble();

    // Set variables to the motor data for kicker follower
    inputs.kickerFollowerConnected =
        BaseStatusSignal.isAllGood(
            kickerFollowerAngularVelocityRotsPerSec,
            kickerFollowerPosition,
            kickerFollowerStatorCurrent,
            kickerFollowerSupplyCurrent,
            kickerFollowerVoltage,
            kickerFollowerTemp);
    inputs.kickerFollowerVelocityRotationsPerSec = kickerFollowerAngularVelocityRotsPerSec.getValueAsDouble();
    inputs.kickerFollowerPositionRots = kickerFollowerPosition.getValueAsDouble();
    inputs.kickerFollowerStatorCurrentAmp = kickerFollowerStatorCurrent.getValueAsDouble();
    inputs.kickerFollowerSupplyCurrentAmp = kickerFollowerSupplyCurrent.getValueAsDouble();
    inputs.kickerFollowerVoltage = kickerFollowerVoltage.getValueAsDouble();
    inputs.kickerFollowerTempC = kickerFollowerTemp.getValueAsDouble();
  }
}
