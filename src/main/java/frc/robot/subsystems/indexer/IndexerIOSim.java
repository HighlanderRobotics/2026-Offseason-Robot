// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

/** Add your docs here. */
public class IndexerIOSim extends IndexerIO {
  TalonFXSimState indexerFxSimState;
  TalonFXSimState kickerFxSimState; // change to kickerFxSimState if problems arise
  TalonFXSimState kickerFxSimStateFollower; // change to kickerFxSimStateFollower is problems arise
  DCMotorSim physicsSimIndexer;
  DCMotorSim physicsSimKicker;

  private final double simLoopPeriod = 0.002; // standard time iirc
  private Notifier simNotifier;
  private double lastSimTime = 0.0;

  public IndexerIOSim(CANBus canBus) {
    super(canBus);

    // every motor gets its own physics sim
    physicsSimIndexer =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.0136, INDEXER_GEAR_RATIO),
            DCMotor.getKrakenX60Foc(1));
    indexerFxSimState = indexerMotor.getSimState();
    indexerFxSimState.setMotorType(MotorType.KrakenX60);
    indexerFxSimState.Orientation = ChassisReference.CounterClockwise_Positive;

    physicsSimKicker =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(2), 0.0136, KICKER_GEAR_RATIO),
            DCMotor.getKrakenX60Foc(2));
    kickerFxSimState = kickerLeaderMotor.getSimState();
    kickerFxSimState.setMotorType(MotorType.KrakenX60);
    kickerFxSimState.Orientation = ChassisReference.CounterClockwise_Positive;

    kickerFxSimStateFollower = kickerFollowerMotor.getSimState();
    kickerFxSimStateFollower.setMotorType(MotorType.KrakenX60);
    kickerFxSimStateFollower.Orientation = ChassisReference.CounterClockwise_Positive;

    // set voltages
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Timer.getTimestamp(); // get current time
              final double deltaTime = currentTime - lastSimTime; // get elapsed time
              lastSimTime = currentTime; // update

              indexerFxSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
              kickerFxSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

              physicsSimIndexer.setInputVoltage(indexerFxSimState.getMotorVoltage());
              physicsSimIndexer.update(deltaTime);

              physicsSimKicker.setInputVoltage(kickerFxSimState.getMotorVoltage());
              physicsSimKicker.update(deltaTime);

              // indexer
              indexerFxSimState.setRawRotorPosition(
                  physicsSimIndexer.getAngularPosition().in(Rotations) * INDEXER_GEAR_RATIO);
              indexerFxSimState.setRotorVelocity(
                  physicsSimIndexer.getAngularVelocity().in(RotationsPerSecond)
                      * INDEXER_GEAR_RATIO);

              // kicker leader
              kickerFxSimState.setRawRotorPosition(
                  physicsSimKicker.getAngularPosition().in(Rotations) * KICKER_GEAR_RATIO);
              kickerFxSimState.setRotorVelocity(
                  physicsSimKicker.getAngularVelocity().in(RotationsPerSecond) * KICKER_GEAR_RATIO);

              // kicker follower
              kickerFxSimStateFollower.setRawRotorPosition(
                  physicsSimKicker.getAngularPosition().in(Rotations) * KICKER_GEAR_RATIO);
              kickerFxSimStateFollower.setRotorVelocity(
                  physicsSimKicker.getAngularVelocity().in(RotationsPerSecond) * KICKER_GEAR_RATIO);
            });
    simNotifier.setName("Indexer");
    simNotifier.startPeriodic(simLoopPeriod);
  }
}
