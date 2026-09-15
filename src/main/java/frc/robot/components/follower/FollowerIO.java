package frc.robot.components.follower;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public class FollowerIO {
  @AutoLog
  public static class FollowerIOInputs {
    public int motorId = 0;
    public double velocityRotPerSec = 0.0;
    public double voltage = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempC = 0.0;
    // For sysid
    public double flywheelPositionRotations = 0.0;
  }

  protected final TalonFX motor;
  private final int motorId;

  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Voltage> voltage;
  private StatusSignal<Current> statorCurrent;
  private StatusSignal<Current> supplyCurrent;
  private StatusSignal<Temperature> temp;
  private StatusSignal<Angle> flywheelPosition;

  private Follower followerReq;

  public FollowerIO(int motorID, int leaderID, MotorAlignmentValue alignment, CANBus canBus) {
    this.motorId = motorID;
    motor = new TalonFX(motorID, canBus);

    velocity = motor.getVelocity();
    voltage = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    temp = motor.getDeviceTemp();
    flywheelPosition = motor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, voltage, statorCurrent, supplyCurrent, temp, flywheelPosition);
    motor.optimizeBusUtilization();

    followerReq = new Follower(leaderID, alignment);
    motor.setControl(followerReq);
  }

  public void updateInputs(FollowerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocity, voltage, statorCurrent, supplyCurrent, temp, flywheelPosition);
    inputs.motorId = motorId;
    inputs.flywheelPositionRotations = flywheelPosition.getValue().in(Rotation);
    inputs.velocityRotPerSec = velocity.getValue().in(Rotation.per(Second));
    inputs.voltage = voltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempC = temp.getValue().in(Celsius);
  }
}
