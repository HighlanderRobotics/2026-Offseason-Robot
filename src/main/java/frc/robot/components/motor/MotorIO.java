package frc.robot.components.motor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public class MotorIO {

  @AutoLog
  public static class MotorIOInputs {

    public double positionRotations = 0.0;
    public double velocityRPS = 0.0;

    public double appliedVoltage = 0.0;
    public double temperatureCelsius = 0.0;

    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;

    public boolean connected = false;
  }

  protected final TalonFX motor;
  private double velocitySetpoint;
  private double positionSetpoint;

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final VelocityVoltage velocityVoltage =
      new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withEnableFOC(true);

  private final StatusSignal<AngularVelocity> velocityRPS;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> statorCurrentAmps;
  private final StatusSignal<Temperature> temperatureCelsius;
  private final StatusSignal<Angle> positionRotations;

  public MotorIO(int motorID, TalonFXConfiguration config, CANBus canbus) {
    motor = new TalonFX(motorID, canbus);

    velocityRPS = motor.getVelocity();
    supplyCurrentAmps = motor.getSupplyCurrent();
    appliedVoltage = motor.getMotorVoltage();
    statorCurrentAmps = motor.getStatorCurrent();
    temperatureCelsius = motor.getDeviceTemp();
    positionRotations = motor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        velocityRPS,
        supplyCurrentAmps,
        statorCurrentAmps,
        appliedVoltage,
        temperatureCelsius,
        positionRotations);

    motor.getConfigurator().apply(config);
    motor.optimizeBusUtilization();
  }

  public void updateInputs(MotorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocityRPS,
        supplyCurrentAmps,
        appliedVoltage,
        statorCurrentAmps,
        temperatureCelsius,
        positionRotations);

    inputs.connected =
        BaseStatusSignal.isAllGood(
            velocityRPS,
            supplyCurrentAmps,
            appliedVoltage,
            statorCurrentAmps,
            temperatureCelsius,
            positionRotations);
    inputs.velocityRPS = velocityRPS.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
    inputs.positionRotations = positionRotations.getValueAsDouble();
  }

  public void setVoltage(double volts) {
    motor.setControl(voltageOut.withOutput(volts));
  }

  public void setVelocity(double velocityRPS) {
    velocitySetpoint = velocityRPS;
    motor.setControl(velocityVoltage.withVelocity(velocityRPS));
  }

  public double getVelocitySetpoint() {
    return velocitySetpoint;
  }

  public void setPositionSetpoint(double positionRotations, double volts) {
    positionSetpoint = positionRotations;

    motor.setControl(motionMagicVoltage.withPosition(positionRotations).withFeedForward(volts));
  }

  public double getPositionSetpoint() {
    return positionSetpoint;
  }

  public void resetPosition(double positionRotations) {
    motor.setPosition(positionRotations);
  }
}
