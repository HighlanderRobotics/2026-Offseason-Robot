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
    public Rotation2d hoodPosition = new Rotation2d();
    public double hoodVelocityRotPerSec = 0.0;
    public double hoodVoltage = 0.0;
    public double hoodStatorCurrentAmps = 0.0;
    public double hoodSupplyCurrentAmps = 0.0;
    public double hoodTempCelsius = 0.0;

    public double hoodPositionRotations = 0.0;
  }

  protected TalonFX hoodMotor;

  private StatusSignal<AngularVelocity> angularVelocity;
  private StatusSignal<Voltage> voltage;
  private StatusSignal<Current> statorCurrent;
  private StatusSignal<Current> supplyCurrent;
  private StatusSignal<Temperature> tempC;
  private StatusSignal<Angle> position;

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withEnableFOC(true);

  private Rotation2d angleSetpoint = new Rotation2d();

  public HoodIO(CANBus canbus) {
    hoodMotor = new TalonFX(DrumSubsystem.HOOD_MOTOR_ID, canbus);

    hoodMotor.getConfigurator().apply(DrumSubsystem.getHoodConfig());

    angularVelocity = hoodMotor.getVelocity();
    voltage = hoodMotor.getMotorVoltage();
    statorCurrent = hoodMotor.getStatorCurrent();
    supplyCurrent = hoodMotor.getSupplyCurrent();
    tempC = hoodMotor.getDeviceTemp();
    position = hoodMotor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, angularVelocity, voltage, statorCurrent, supplyCurrent, tempC, position);
    hoodMotor.optimizeBusUtilization();
  }

  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        angularVelocity, voltage, statorCurrent, supplyCurrent, tempC, position);
    inputs.connected =
        BaseStatusSignal.isAllGood(
            angularVelocity, voltage, statorCurrent, supplyCurrent, tempC, position);
    inputs.hoodPosition = new Rotation2d(position.getValue()); // WPILIB handles units
    inputs.hoodVelocityRotPerSec = angularVelocity.getValue().in(RotationsPerSecond);
    inputs.hoodVoltage = voltage.getValueAsDouble();
    inputs.hoodStatorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.hoodSupplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.hoodTempCelsius = tempC.getValueAsDouble();
    inputs.hoodPositionRotations = position.getValue().in(Rotation);
  }

  public void setVoltage(double volts) {
    hoodMotor.setControl(voltageOut.withOutput(volts));
  }

  public void setPositionSetpoint(Rotation2d angle) {
    this.angleSetpoint = angle;
    hoodMotor.setControl(
        motionMagicVoltage.withPosition(angle.getMeasure())); // WPILIB handles units
  }

  public void rezeroMotor(Rotation2d position) {
    hoodMotor.setPosition(position.getMeasure());
  }

  public void rezeroMotorToBottom() {
    System.out.println("Rezeroing Hood Motor to Bottom");
    rezeroMotor(DrumSubsystem.HOOD_MIN_ANGLE);
  }

  public Rotation2d getAngleSetpoint() {
    return angleSetpoint;
  }
}
