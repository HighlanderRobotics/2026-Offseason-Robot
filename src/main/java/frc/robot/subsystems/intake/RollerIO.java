// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

/**
Things to log
velocity rots per second
applied voltage
position rotations
supply current
stator current
temp
connected
 */
public class RollerIO {
    @AutoLog
    public static class RollerIOInputs {
        public double velocityRotsPerSecond = 0.0;
        public double appliedVoltage = 0.0;
        public double positionRotations = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double tempatureCelsius = 0.0;
        public boolean connected = false;
    }

    protected final TalonFX motor;

    private final StatusSignal<AngularVelocity> AngularVelocityRotsPerSecond;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrentAmps;
    private final StatusSignal<Current> statorCurrentAmps;
    private final StatusSignal<Temperature> tempatureCelsius;
    private final StatusSignal<Angle> positionRotations;

    private double setpointVelocity;
    
    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);
}
