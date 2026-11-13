package frc.robot.subsystems.drum.flywheel;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

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

    // Leader
    public double angularVelocityRotsPerSec = 0.0;
    public double voltage = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double flywheelPositionRotations = 0.0;

    // Followers
    public double[] followerVelocityRotsPerSec = new double[3];
    public double[] followerVoltage = new double[3];
    public double[] followerStatorCurrentAmps = new double[3];
    public double[] followerSupplyCurrentAmps = new double[3];
    public double[] followerTempCelsius = new double[3];
    public double[] followerPositionRotations = new double[3];
}
  public class FollowerStatusSignals {
     public StatusSignal<AngularVelocity> velocity;
     public StatusSignal<Voltage> voltage;
     public StatusSignal<Current> statorCurrent;
     public StatusSignal<Current> supplyCurrent;
     public StatusSignal<Temperature> temp;
     public StatusSignal<Angle> flywheelPos;
  }
  private Follower followerReq;

  protected final TalonFX leader;
  protected TalonFX followerTalons[] = new TalonFX[3];
  protected FollowerStatusSignals followerStatusSignals[] = new FollowerStatusSignals[3];


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
    // Initialize follower Talons
    followerReq = new Follower(DrumSubsystem.FLYWHEEL_LEADER_ID, MotorAlignmentValue.Opposed);

    for (int i = 0; i < followerTalons.length; i++) {
      followerTalons[i] = new TalonFX(DrumSubsystem.FLYWHEEL_LEADER_ID + i + 1, canbus);
      followerTalons[i].getConfigurator().apply(DrumSubsystem.getFlywheelConfig());
      followerTalons[i].optimizeBusUtilization();
      followerTalons[i].setControl(followerReq);
      FollowerStatusSignals statusSignals = new FollowerStatusSignals();

      statusSignals.velocity = followerTalons[i].getVelocity();
      statusSignals.voltage = followerTalons[i].getMotorVoltage();
      statusSignals.statorCurrent = followerTalons[i].getStatorCurrent();
      statusSignals.supplyCurrent = followerTalons[i].getSupplyCurrent();
      statusSignals.temp = followerTalons[i].getDeviceTemp();
      statusSignals.flywheelPos = followerTalons[i].getPosition();
      BaseStatusSignal.setUpdateFrequencyForAll(50.0,
        statusSignals.velocity, statusSignals.voltage, statusSignals.statorCurrent,
        statusSignals.supplyCurrent, statusSignals.temp, statusSignals.flywheelPos);
      followerStatusSignals[i] = statusSignals;

    }
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
    for (int i = 0; i < followerTalons.length; i++) {
    BaseStatusSignal.refreshAll(
        followerStatusSignals[i].velocity,
        followerStatusSignals[i].voltage,
        followerStatusSignals[i].statorCurrent,
        followerStatusSignals[i].supplyCurrent,
        followerStatusSignals[i].temp,
        followerStatusSignals[i].flywheelPos
    );

    inputs.connected &= BaseStatusSignal.isAllGood(
        followerStatusSignals[i].velocity,
        followerStatusSignals[i].voltage,
        followerStatusSignals[i].statorCurrent,
        followerStatusSignals[i].supplyCurrent,
        followerStatusSignals[i].temp,
        followerStatusSignals[i].flywheelPos
    );

    inputs.followerVelocityRotsPerSec[i] =
        followerStatusSignals[i].velocity.getValue().in(Rotation.per(Second));

    inputs.followerVoltage[i] =
        followerStatusSignals[i].voltage.getValueAsDouble();

    inputs.followerStatorCurrentAmps[i] =
        followerStatusSignals[i].statorCurrent.getValueAsDouble();

    inputs.followerSupplyCurrentAmps[i] =
        followerStatusSignals[i].supplyCurrent.getValueAsDouble();

    inputs.followerTempCelsius[i] =
        followerStatusSignals[i].temp.getValue().in(Celsius);

    inputs.followerPositionRotations[i] =
        followerStatusSignals[i].flywheelPos.getValue().in(Rotation);
}
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
