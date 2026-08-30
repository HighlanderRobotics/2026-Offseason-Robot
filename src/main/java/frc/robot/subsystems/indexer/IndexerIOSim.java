package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim extends IndexerIO {
  TalonFXSimState indexerFxSimState;
  TalonFXSimState kickerFxSimState;
  DCMotorSim physicsSimIndexer;
  DCMotorSim physicsSimKicker;

  private final double simLoopPeriod = 0.002;
  private Notifier simNotifier;
  private double lastSimTime = 0.0;

  public IndexerIOSim(CANBus canbus) {
    super(canbus);

    // Define physics simulations for each motor
    physicsSimIndexer =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.0136, GEAR_RATIO),
            DCMotor.getKrakenX60Foc(1));
    indexerFxSimState = indexerMotor.getSimState();
    indexerFxSimState.setMotorType(MotorType.KrakenX60);
    indexerFxSimState.Orientation = ChassisReference.CounterClockwise_Positive;

    physicsSimKicker =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.0136, GEAR_RATIO),
            DCMotor.getKrakenX60Foc(1));
    kickerFxSimState = kickerMotor.getSimState();
    kickerFxSimState.setMotorType(MotorType.KrakenX60);
    kickerFxSimState.Orientation = ChassisReference.CounterClockwise_Positive;

    // Set voltages and calculate position and velocity in sim for both motors
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Timer.getFPGATimestamp();
              final double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              indexerFxSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
              kickerFxSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

              physicsSimIndexer.setInputVoltage(indexerFxSimState.getMotorVoltage());
              physicsSimIndexer.update(deltaTime);

              physicsSimKicker.setInputVoltage(kickerFxSimState.getMotorVoltage());
              physicsSimKicker.update(deltaTime);

              indexerFxSimState.setRawRotorPosition(
                  physicsSimIndexer.getAngularPosition().in(Rotations) * GEAR_RATIO);
              indexerFxSimState.setRotorVelocity(
                  physicsSimIndexer.getAngularVelocity().in(RotationsPerSecond) * GEAR_RATIO);
            
              kickerFxSimState.setRawRotorPosition(
                  physicsSimKicker.getAngularPosition().in(Rotations) * KICKER_GEAR_RATIO);
              kickerFxSimState.setRotorVelocity(
                  physicsSimKicker.getAngularVelocity().in(RotationsPerSecond) * KICKER_GEAR_RATIO);
            });

    simNotifier.startPeriodic(simLoopPeriod);
  }
}
