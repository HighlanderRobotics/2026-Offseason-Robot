package frc.robot.subsystems.drum.flywheel;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Second;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drum.DrumSubsystem;
import org.littletonrobotics.junction.AutoLog;

public class FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean connected = false;
    public double angularVelocityRotsPerSec = 0.0;
    public double voltage = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double flywheelPositionRotations = 0.0;
  }

  protected final TalonFX leader;

  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Voltage> voltage;
  private StatusSignal<Current> statorCurrent;
  private StatusSignal<Current> supplyCurrent;
  private StatusSignal<Temperature> temp;
  private StatusSignal<Angle> flywheelPos;

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private MotionMagicVelocityVoltage motionMagic = 
    new MotionMagicVelocityVoltage(0.0).withEnableFOC(true); 
  
  private double velocitySetpointRotPerSec = Double.NaN;

  public FlywheelIO(CANBus canbus) {
    leader = new TalonFX(DrumSubsystem.FLYWHEEL_LEADER_ID,canbus);
    leader.getConfigurator().apply(DrumSubsystem.getFlywheelConfig());

    velocity = leader.getVelocity();
    voltage = leader.getMotorVoltage();
    statorCurrent = leader.getStatorCurrent();
    supplyCurrent = leader.getSupplyCurrent();
    temp = leader.getDeviceTemp();
    flywheelPos = leader.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, 
      velocity, voltage, statorCurrent, supplyCurrent, temp, flywheelPos);
    leader.optimizeBusUtilization();
  }
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
      velocity, voltage, statorCurrent, supplyCurrent, temp, flywheelPos
    );
    inputs.connected = BaseStatusSignal.isAllGood(
      velocity, voltage, statorCurrent, supplyCurrent, temp, flywheelPos);
    inputs.flywheelPositionRotations = flywheelPos.getValue().in(Rotation);
    inputs.angularVelocityRotsPerSec = velocity.getValue().in(Rotation.per(Second));
    inputs.voltage = voltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValue().in(Celsius);
  }
  public void setVoltage(double volts) {
    voltageOut = voltageOut.withOutput(volts);
    leader.setControl(voltageOut);
  }
  
  public void setVelocitySetpoint(double velocityRotPerSec) {
    velocitySetpointRotPerSec = velocityRotPerSec;
    leader.setControl(motionMagic.withVelocity(velocityRotPerSec));
  }

  public double getSetpointRotPerSec() {
    return velocitySetpointRotPerSec;
  }
}
