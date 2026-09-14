package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.motor.MotorIO;

public class ArmSubsystem extends SubsystemBase {
  public static double ARM_GEAR_RATIO = 10;

  private final MotorIO motor;
  private final MotorIO.MotorIOInputs inputs = new MotorIO.MotorIOInputs();

  private Rotation2d setpoint = new Rotation2d();

  public ArmSubsystem(MotorIO motor) {
    this.motor = motor;
  }

  public static TalonFXConfiguration getConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Feedback.SensorToMechanismRatio = ArmSubsystem.ARM_GEAR_RATIO;

    config.Slot0.kS = 0.43477;
    config.Slot0.kV = 0.144;
    config.Slot0.kA = 0.016433;
    config.Slot0.kP = 0.37;
    config.Slot0.kD = 0.0;

    config.CurrentLimits.StatorCurrentLimit = 70.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;

    config.MotionMagic.MotionMagicAcceleration = 100.0;

    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    return config;
  }

  public static DCMotorSim getSim() {
    return new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44Foc(1), 0.00001, ArmSubsystem.ARM_GEAR_RATIO),
        DCMotor.getKrakenX44Foc(1));
  }

  public void setAngle(Rotation2d angle) {
    this.setpoint = angle;

    motor.setPositionSetpoint(angle.getRotations(), 0.0);
  }

  public void setAngle(Rotation2d angle, double ffVolts) {
    this.setpoint = angle;

    motor.setPositionSetpoint(angle.getRotations(), ffVolts);
  }

  public Rotation2d getSetpoint() {
    return setpoint;
  }

  public void resetEncoder(Rotation2d newPosition) {
    motor.resetPosition(newPosition.getRotations());
  }

  @Override
  public void periodic() {
    motor.updateInputs(inputs);
  }
}
