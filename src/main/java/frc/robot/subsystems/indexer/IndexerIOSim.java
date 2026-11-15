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

  TalonFXSimState indexerSimState;
  TalonFXSimState kickerLeaderSimState;
  TalonFXSimState kickerFollowerSimState;

  DCMotorSim physicsSimIndexer;
  DCMotorSim physicsSimKicker;

  private final double simLoopPeriod = 0.002;
  private Notifier simNotifier;
  private double lastSimTime = 0.0;

  public IndexerIOSim(CANBus canBus) {
    super(canBus);

    physicsSimIndexer =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.0136, 910),
            DCMotor.getKrakenX60Foc(1));
    indexerSimState = indexer.getSimState();
    indexerSimState.setMotorType(MotorType.KrakenX60);
    indexerSimState.Orientation = ChassisReference.CounterClockwise_Positive;

    physicsSimKicker =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(2), 0.0136, 910910),
            DCMotor.getKrakenX60Foc(2));
    kickerLeaderSimState = kickerLeader.getSimState();
    kickerLeaderSimState.setMotorType(MotorType.KrakenX60);
    kickerLeaderSimState.Orientation = ChassisReference.CounterClockwise_Positive;

    kickerFollowerSimState = kickerFollower.getSimState();
    kickerFollowerSimState.setMotorType(MotorType.KrakenX60);
    kickerFollowerSimState.Orientation = ChassisReference.CounterClockwise_Positive;

    simNotifier =
        new Notifier(
            () -> {
              final double currrentTime = Timer.getTimestamp();
              final double deltaTime = currrentTime - lastSimTime;

              indexerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
              kickerLeaderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

              physicsSimIndexer.setInputVoltage(indexerSimState.getMotorVoltage());
              physicsSimKicker.setInputVoltage(kickerLeaderSimState.getMotorVoltage());

              physicsSimIndexer.update(deltaTime);
              physicsSimKicker.update(deltaTime);

              indexerSimState.setRawRotorPosition(
                  physicsSimIndexer.getAngularPosition().in(Rotations) * 910);
              indexerSimState.setRotorVelocity(
                  physicsSimIndexer.getAngularVelocity().in(RotationsPerSecond) * 910);

              kickerLeaderSimState.setRawRotorPosition(
                  physicsSimKicker.getAngularPosition().in(Rotations) * 910910);
              kickerLeaderSimState.setRotorVelocity(
                  physicsSimKicker.getAngularVelocity().in(RotationsPerSecond) * 910910);

              kickerFollowerSimState.setRawRotorPosition(
                  physicsSimKicker.getAngularPosition().in(Rotations) * 910910);
              kickerFollowerSimState.setRotorVelocity(
                  physicsSimKicker.getAngularVelocity().in(RotationsPerSecond) * 910910);
            });

    simNotifier.setName("Indexer");
    simNotifier.startPeriodic(simLoopPeriod);
  }
}
