package frc.robot.subsystems.drum.hood;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drum.DrumSubsystem;
import org.littletonrobotics.junction.AutoLog;

public class HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public boolean connected = false;
    public Rotation2d position = new Rotation2d();
    public double angularVelocityRotPerSec = 0.0;
    public double voltage = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempC = 0.0;
    // For sysid
    public double positionRotations = 0.0;
  }

  protected TalonFX motor;

  private StatusSignal<AngularVelocity> angularVelocity;
  private StatusSignal<Voltage> voltage;
  private StatusSignal<Current> statorCurrent;
  private StatusSignal<Current> supplyCurrent;
  private StatusSignal<Temperature> temp;
  private StatusSignal<Angle> position;

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withEnableFOC(true);

  private Rotation2d angleSetpoint = new Rotation2d();

  public HoodIO(CANBus canBus) {
    motor = new TalonFX(13, canBus); // TODO: REAL ID

    motor.getConfigurator().apply(DrumSubsystem.getHoodConfig());

    angularVelocity = motor.getVelocity();
    voltage = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    temp = motor.getDeviceTemp();
    position = motor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, angularVelocity, voltage, statorCurrent, supplyCurrent, temp, position);
    motor.optimizeBusUtilization();
  }

  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        angularVelocity, voltage, statorCurrent, supplyCurrent, temp, position);

    inputs.connected =
        BaseStatusSignal.isAllGood(
            angularVelocity, voltage, statorCurrent, supplyCurrent, temp, position);
    inputs.position = new Rotation2d(position.getValue()); // WPIlib handles units here
    inputs.angularVelocityRotPerSec = angularVelocity.getValue().in(RotationsPerSecond);
    inputs.voltage = voltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.tempC = temp.getValueAsDouble();
    inputs.positionRotations = position.getValue().in(Rotation);
  }

  public void setVoltage(double volts) {
    motor.setControl(voltageOut.withOutput(volts));
  }

  public void setPositionSetpoint(Rotation2d setpoint) {
    this.angleSetpoint = setpoint;
    motor.setControl(
        motionMagicVoltage.withPosition(setpoint.getMeasure())); // WPILib handles units
  }

  public Rotation2d getAngleSetpoint() {
    return angleSetpoint;
  }
}
