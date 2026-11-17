// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class RollerIOSim extends RollerIO {
    private final DCMotorSim rollerSim;
    private TalonFXSimState talonSim;
    private double lastLoopTime = 0.0;
    Notifier notifier;

    public RollerIOSim(int motorID, TalonFXConfiguration config, CANBus canBus) {
        super(motorID, config, canBus);
        rollerSim =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(2), 0.00001, SlapdownSubsystem.ROLLER_GEAR_RATIO),
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
                    rollerSim.getAngularPositionRotations() * SlapdownSubsystem.ROLLER_GEAR_RATIO);
                talonSim.setRotorVelocity(
                    rollerSim.getAngularVelocity().in(RotationsPerSecond)
                        * SlapdownSubsystem.ROLLER_GEAR_RATIO);
                });
        notifier.startPeriodic(0.002);
    }
}
