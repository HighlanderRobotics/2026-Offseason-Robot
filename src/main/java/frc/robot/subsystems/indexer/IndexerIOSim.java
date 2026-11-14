// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class IndexerIOSim extends IndexerIO {
  TalonFXSimState indexerFxSimState;
  TalonFXSimState kickerLeaderFxSimState; // change to kickerFxSimState if problems arise
  TalonFXSimState kickerFollowerFxSimState; // change to kickerFxSimStateFollower is problems arise
  DCMotorSim physicsSimIndexer;
  DCMotorSim physiscsSimKicker;

  private final double simLoopPeriod = 0.002; // standard time iirc
  private Notifier simNotifier;
  private double lastSimTime = 0.0;
  public IndexerIOSim(CANBus canBus) {
    super(canBus);
  }
}
