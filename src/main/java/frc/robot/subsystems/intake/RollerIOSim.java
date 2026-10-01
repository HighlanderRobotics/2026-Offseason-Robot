package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerIOSim extends RollerIO {
  private final DCMotorSim rollerSim;
  private TalonFXSimState talonSim;
  private double lastLoopTime = 0.0;
  // TODO: Find last loop time
  Notifier notifier;

  public RollerIOSim(int motorID, TalonFXConfiguration config, CANBus canbus) {

    super(motorID, config, canbus);
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(2), 0.001, SlapdownSubsystem.ROLLER_GEAR_RATIO),
            DCMotor.getKrakenX60Foc(2));
    talonSim = motor.getSimState();
    talonSim.setMotorType(MotorType.KrakenX60);

    notifier =
        new Notifier(
            () -> {
              double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = (currentTime - lastLoopTime);
              lastLoopTime = currentTime;

              talonSim.setSupplyVoltage(RobotController.getBatteryVoltage());

              rollerSim.setInputVoltage(talonSim.getMotorVoltage());
              rollerSim.update(deltaTime);
              talonSim.setRawRotorPosition(
                  rollerSim.getAngularPositionRotations() * rollerSim.getGearing());
              talonSim.setRotorVelocity(
                  (rollerSim.getAngularVelocityRPM() * 60) * rollerSim.getGearing());
            });
    notifier.startPeriodic(0.002);
  }
}
