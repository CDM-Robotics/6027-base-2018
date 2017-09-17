package com.team6072.frc2018.auto.actions;

import edu.wpi.first.wpilibj.Timer;

import com.team6072.frc2018.subsystems.Intake;

/**
 * Deploys the intake. Based on the runIntake parameter, waits 0.25 seconds to let the intake deploy reliably.
 * 
 * @see Action
 */
public class DeployIntakeAction implements Action {

    Intake mIntake = Intake.getInstance();
    double startTime;
    boolean runIntake;

    public DeployIntakeAction() {
        runIntake = false;
    }

    public DeployIntakeAction(boolean runIntake) {
        this.runIntake = runIntake;
    }

    @Override
    public boolean isFinished() {
        if (runIntake) {
            return Timer.getFPGATimestamp() - startTime > 0.25;
        } else {
            return true;
        }
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        if (runIntake) {
            // mIntake.setOff();
        }
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
        mIntake.deploy();
        if (runIntake)
            mIntake.setOn();
    }
}
