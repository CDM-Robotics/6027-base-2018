package com.team6072.frc2018.paths;

import com.team6072.frc2018.auto.modes.GearThenHopperShootModeBlue;
import com.team6072.frc2018.paths.profiles.PathAdapter;
import com.team6072.lib.util.control.Path;
import com.team6072.lib.util.math.RigidTransform2d;

/**
 * Path from the blue alliance wall to the blue boiler peg.
 * 
 * Used in GearThenHopperShootModeBlue
 * 
 * @see GearThenHopperShootModeBlue
 * @see PathContainer
 */
public class StartToBoilerGearBlue implements PathContainer {

    @Override
    public Path buildPath() {
        return PathAdapter.getBlueGearPath();
    }

    @Override
    public RigidTransform2d getStartPose() {
        return PathAdapter.getBlueStartPose();
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}