package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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

    public double kickerVelocityRotationsPerSec = 0.0;
    public double kickerPositionRots = 0.0;
    public double kickerStatorCurrentAmp = 0.0;
    public double kickerSupplyCurrentAmp = 0.0;
    public double kickerVoltage = 0.0;
    public double kickerTempC = 0.0;
    public boolean kickerConnected = false;
  }

  // TODO: Set gear ratios
  public static final double GEAR_RATIO = 1.0;
  public static final double KICKER_GEAR_RATIO = 1.0;

  protected final TalonFX indexerMotor;
  protected final TalonFX kickerMotor;

  // StatusSignal - used to get information about each motor
  private final StatusSignal<AngularVelocity> indexerAngularVelocityRotsPerSec;
  private final StatusSignal<Angle> indexerPosition;
  private final StatusSignal<Current> indexerStatorCurrent;
  private final StatusSignal<Current> indexerSupplyCurrent;
  private final StatusSignal<Voltage> indexerVoltage;
  private final StatusSignal<Temperature> indexerTemp;

  private final StatusSignal<AngularVelocity> kickerAngularVelocityRotsPerSec;
  private final StatusSignal<Angle> kickerPosition;
  private final StatusSignal<Current> kickerStatorCurrent;
  private final StatusSignal<Current> kickerSupplyCurrent;
  private final StatusSignal<Voltage> kickerVoltage;
  private final StatusSignal<Temperature> kickerTemp;

  // Voltage and velocity controllers
  private VoltageOut voltageOut =
      new VoltageOut(0.0).withEnableFOC((true)); // FOC - higher power and smoother
  private VelocityVoltage velocityVoltage =
      new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);

  public IndexerIO(CANBus canBus) {
    // TODO: set motor ID for indexer
    indexerMotor = new TalonFX(16, canBus);
    indexerMotor.getConfigurator().apply(IndexerIO.getIndexerConfiguration());

    // TODO: set motor ID for kicker
    kickerMotor = new TalonFX(15, canBus);
    kickerMotor.getConfigurator().apply(IndexerIO.getKickerConfiguration());

    // Set the data for each motor
    indexerAngularVelocityRotsPerSec = indexerMotor.getVelocity();
    indexerPosition = indexerMotor.getPosition();
    indexerStatorCurrent = indexerMotor.getStatorCurrent();
    indexerSupplyCurrent = indexerMotor.getSupplyCurrent();
    indexerVoltage = indexerMotor.getMotorVoltage();
    indexerTemp = indexerMotor.getDeviceTemp();

    kickerAngularVelocityRotsPerSec = kickerMotor.getVelocity();
    kickerPosition = kickerMotor.getPosition();
    kickerStatorCurrent = kickerMotor.getStatorCurrent();
    kickerSupplyCurrent = kickerMotor.getSupplyCurrent();
    kickerVoltage = kickerMotor.getMotorVoltage();
    kickerTemp = kickerMotor.getDeviceTemp();

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
        kickerAngularVelocityRotsPerSec,
        kickerPosition,
        kickerStatorCurrent,
        kickerSupplyCurrent,
        kickerStatorCurrent,
        kickerVoltage,
        kickerTemp);
    indexerMotor
        .optimizeBusUtilization(); // only update variables that have update frequency set to
    // non-zero value
    kickerMotor.optimizeBusUtilization();
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
    kickerMotor.setControl(voltageOut.withOutput(voltage));
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
        kickerAngularVelocityRotsPerSec,
        kickerPosition,
        kickerStatorCurrent,
        kickerSupplyCurrent,
        kickerVoltage,
        kickerTemp);

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

    // Set variables to the motor data for kicker
    inputs.kickerConnected =
        BaseStatusSignal.isAllGood(
            kickerAngularVelocityRotsPerSec,
            kickerPosition,
            kickerStatorCurrent,
            kickerSupplyCurrent,
            kickerVoltage,
            kickerTemp);
    inputs.kickerVelocityRotationsPerSec = kickerAngularVelocityRotsPerSec.getValueAsDouble();
    inputs.kickerPositionRots = kickerPosition.getValueAsDouble();
    inputs.kickerStatorCurrentAmp = kickerStatorCurrent.getValueAsDouble();
    inputs.kickerSupplyCurrentAmp = kickerSupplyCurrent.getValueAsDouble();
    inputs.kickerVoltage = kickerVoltage.getValueAsDouble();
    inputs.kickerTempC = kickerTemp.getValueAsDouble();
  }
}
