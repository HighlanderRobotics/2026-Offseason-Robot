package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.motor.MotorIO;
import frc.robot.components.motor.MotorIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class RollerSubsystem extends SubsystemBase {

  public static final double GEAR_RATIO = 10.0;

  private final MotorIO motor;
  private final MotorIO follower;

  private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

  private double velocitySetpointRPS = 0.0;

  public RollerSubsystem(MotorIO motor, MotorIO follower) {
    this.motor = motor;
    this.follower = follower;
  }

  public static TalonFXConfiguration getConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    // Velocity PID and feedforward gains.
    // Replace these with values tuned for this roller.
    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 0.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.CurrentLimits.StatorCurrentLimit = 70.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;

    return config;
  }

  public static DCMotorSim getSim() {
    DCMotor motor = DCMotor.getKrakenX44Foc(1);

    return new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.00001, GEAR_RATIO), motor);
  }

  public void setVelocity(double targetVelocityRPS) {
    velocitySetpointRPS = targetVelocityRPS;
    motor.setVelocity(targetVelocityRPS);
  }

  public double getSetpoint() {
    return velocitySetpointRPS;
  }

  public double getVelocity() {
    return inputs.velocityRPS;
  }

  public void stop() {
    setVelocity(0.0);
  }

  @Override
  public void periodic() {
    motor.updateInputs(inputs);
    Commands.run(() -> setVelocity(10));
    // Logger.processInputs("Roller/Motor", inputs);
    Logger.processInputs("Roller", inputs);
    // TODO
    // Logger.recordOutput("Roller/VelocitySetpointRPS", velocitySetpointRPS);
  }
}
