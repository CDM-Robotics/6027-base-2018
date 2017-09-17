package com.team6072.frc2018.auto.modes;

import com.team6072.frc2018.auto.AutoModeBase;
import com.team6072.frc2018.auto.AutoModeEndedException;
import com.team6072.frc2018.auto.actions.BeginShootingAction;
import com.team6072.frc2018.auto.actions.DeployIntakeAction;
import com.team6072.frc2018.auto.actions.DrivePathAction;
import com.team6072.frc2018.auto.actions.EndShootingAction;
import com.team6072.frc2018.auto.actions.ResetPoseFromPathAction;
import com.team6072.frc2018.auto.actions.ScoreGearAction;
import com.team6072.frc2018.auto.actions.WaitAction;
import com.team6072.frc2018.paths.BoilerGearToShootBlue;
import com.team6072.frc2018.paths.PathContainer;
import com.team6072.frc2018.paths.StartToBoilerGearBlue;

/**
 * Scores the preload gear onto the boiler-side peg then shoots the 10 preloaded fuel
 * 
 * @see AutoModeBase
 */
public class BoilerGearThenShootModeBlue extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitAction(2));
        PathContainer gearPath = new StartToBoilerGearBlue();
        runAction(new ResetPoseFromPathAction(gearPath));
        runAction(new DrivePathAction(gearPath));
        runAction(new DeployIntakeAction());
        runAction(new ScoreGearAction());
        runAction(new DrivePathAction(new BoilerGearToShootBlue()));
        runAction(new BeginShootingAction());
        runAction(new WaitAction(15));
        runAction(new EndShootingAction());
    }
}
