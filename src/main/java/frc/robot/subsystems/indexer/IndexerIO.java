// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.*;

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
    }
}
 