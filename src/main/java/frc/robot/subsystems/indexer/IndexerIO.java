// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

/** Things to log
temp
voltage
connected?
stator amps
supply amps
position for rots
velocity rots
 */
public class IndexerIO {
    @AutoLog 
    public static class IndexerIOInputs {
        // main motor
        public double indexerVelocityRotPerSecond = 0.0;
        public double indexerPositonRots = 0.0;
        public double indexerStatorAmps = 0.0;
        public double indexerSupplyAmps = 0.0;
        public double indexerTempatureCelcius = 0.0;
        public boolean indexerConnected = false; // i think it updates in the updateinputs

        // kicker leader
        public double kickerLeaderVelocityRotPerSecond = 0.0;
        public double kickerLeaderPositionRots = 0.0;
        public double kickerLeaderStatorAmps = 0.0;
        public double kickerLeaderSupplyAmps = 0.0;
        public double kickerLeaderTempatureCelcius = 0.0;
        public boolean kickerLeaderConnected = false;

        // kicker follower
        public double kickerFollowerVelocityRotPerSecond = 0.0;
        public double kickerFollowerPositionRots = 0.0;
        public double kickerFollowerStatorAmps = 0.0;
        public double kickerFollowerSupplyAmps = 0.0;
        public double kickerFollowerTempatureCelcius = 0.0;
        public boolean kickerFollowerConnected = false;

        // gear ratios
        public static final double INDEXER_GEAR_RATIO = 50.0/12.0;
        public static final double KICKER_GEAR_RATIO = 30.0/12.0;

        // make motors
        protected final TalonFX indexerMotor;
        protected final TalonFX kickerLeaderMotor;
        protected final TalonFX kickerFollowerMotor;

        // indexer status signals
        private final StatusSignal<AngularVelocity> indexerAngularVelocityRotsPerSec;
        private final StatusSignal<Angle> indexerPosition;
        private final StatusSignal<Current> indexerStatorCurrent;
        private final StatusSignal<Current> indexerSupplyCurrent;
        private final StatusSignal<Voltage> indexerVoltage;
        private final StatusSignal<Temperature> indexerTempature;

        // kicker leader status signals
        private final StatusSignal<AngularVelocity> kickerLeaderAngularVelocityRotsPerSec;
        private final StatusSignal<Angle> kickerLeaderPosition;
        private final StatusSignal<Current> kickerLeaderStatorCurrent;
        private final StatusSignal<Current> kickerLeaderSupplyCurrent;
        private final StatusSignal<Voltage> kickerLeaderVoltage;
        private final StatusSignal<Temperature> kickerLeaderTempature;

        // kicker follower status signals
        private final StatusSignal<AngularVelocity> kickerFollowerAngularVelocityRotsPerSec;
        private final StatusSignal<Angle> kickerFollowerPosition;
        private final StatusSignal<Current> kickerFollowerStatorCurrent;
        private final StatusSignal<Current> kickerFollowerSupplyCurrent;
        private final StatusSignal<Voltage> kickerFollowerVoltage;
        private final StatusSignal<Temperature> kickerFollowerTempature;

        // voltage outs
        private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC((true));
        private VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);
    }
}
 