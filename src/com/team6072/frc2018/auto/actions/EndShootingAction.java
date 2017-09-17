package com.team6072.frc2018.auto.actions;

import com.team6072.frc2018.subsystems.Superstructure;

/**
 * Action to make the robot stop shooting
 * 
 * @see Action
 * @see RunOnceAction
 */
public class EndShootingAction extends RunOnceAction implements Action {

    @Override
    public void runOnce() {
        Superstructure.getInstance().setWantedState(Superstructure.WantedState.IDLE);
    }

}
