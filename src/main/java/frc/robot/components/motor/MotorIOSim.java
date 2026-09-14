package frc.robot.components.motor;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class MotorIOSim extends MotorIO {
  private final DCMotorSim motorSim;
  private TalonFXSimState talonSim;
  private double lastLoopTime = 0.0;
  Notifier notifier;

  public MotorIOSim(
      int motorID,
      TalonFXConfiguration config,
      DCMotorSim motorSim,
      MotorType motorType,
      CANBus canbus) {

    super(motorID, config, canbus);
    this.motorSim = motorSim;
    talonSim = motor.getSimState();
    talonSim.setMotorType(motorType);

    notifier =
        new Notifier(
            () -> {
              double deltaTime = (Utils.getCurrentTimeSeconds() - lastLoopTime);
              lastLoopTime = Utils.getCurrentTimeSeconds();
              talonSim.setSupplyVoltage(RobotController.getBatteryVoltage());
              motorSim.setInputVoltage(talonSim.getMotorVoltage());
              motorSim.update(deltaTime);
              talonSim.setRawRotorPosition(
                  motorSim.getAngularPositionRotations() * motorSim.getGearing());
              talonSim.setRotorVelocity(
                  (motorSim.getAngularVelocityRPM() / 60) * motorSim.getGearing());
            });
    notifier.startPeriodic(0.002);
  }
}
