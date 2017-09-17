package com.team6072.frc2018.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team6072.frc2018.Constants;
import com.team6072.frc2018.Robot;
import com.team6072.frc2018.RobotState;
import com.team6072.frc2018.ShooterAimingParameters;
import com.team6072.frc2018.loops.Loop;
import com.team6072.frc2018.loops.Looper;
import com.team6072.lib.util.CircularBuffer;
import com.team6072.lib.util.InterpolatingDouble;
import com.team6072.lib.util.drivers.RevRoboticsAirPressureSensor;

import java.util.Optional;

/**
 * The superstructure subsystem is the overarching superclass containing all components of the superstructure: the
 * intake, hopper, feeder, shooter and LEDs. The superstructure subsystem also contains some miscellaneous hardware that
 * is located in the superstructure but isn't part of any other subsystems like the compressor, pressure sensor, and
 * hopper wall pistons.
 * 
 * Instead of interacting with subsystems like the feeder and intake directly, the {@link Robot} class interacts with
 * the superstructure, which passes on the commands to the correct subsystem.
 * 
 * The superstructure also coordinates actions between different subsystems like the feeder and shooter.
 *
 * @see LED
 * @see Subsystem
 */
public class Superstructure extends Subsystem {

    static Superstructure mInstance = null;

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }


    private final LED mLED = LED.getInstance();
    private final Solenoid mHopperSolenoid = Constants.makeSolenoidForId(Constants.kHopperSolenoidId);
    private final Compressor mCompressor = new Compressor(0);
    private final RevRoboticsAirPressureSensor mAirPressureSensor = new RevRoboticsAirPressureSensor(3);

    // Superstructure doesn't own the drive, but needs to access it
    private final Drive mDrive = Drive.getInstance();

    // Intenal state of the system
    public enum SystemState {
        IDLE,
        WAITING_FOR_ALIGNMENT, // waiting for the drivebase to aim
        WAITING_FOR_FLYWHEEL, // waiting for the shooter to spin up
        SHOOTING, // shooting
        SHOOTING_SPIN_DOWN, // short period after the driver releases the shoot button where the flywheel
                            // continues to spin so the last couple of shots don't go short
        UNJAMMING, // unjamming the feeder and hopper
        UNJAMMING_WITH_SHOOT, // unjamming while the flywheel spins
        JUST_FEED, // run hopper and feeder but not the shooter
        EXHAUSTING, // exhaust the feeder, hopper, and intake
        HANGING, // run shooter in reverse, everything else is idle
        RANGE_FINDING // blink the LED strip to let drivers know if they are at an optimal shooting range
    };

    // Desired function from user
    public enum WantedState {
        IDLE, SHOOT, UNJAM, UNJAM_SHOOT, MANUAL_FEED, EXHAUST, HANG, RANGE_FINDING
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    private double mCurrentTuningRpm = Constants.kShooterTuningRpmFloor;
    private double mLastGoalRange = 0.0;

    private boolean mCompressorOverride = false;

    private CircularBuffer mShooterRpmBuffer = new CircularBuffer(Constants.kShooterJamBufferSize);
    private double mLastDisturbanceShooterTime;
    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    public boolean isDriveOnTarget() {
        return mDrive.isOnTarget() && mDrive.isAutoAiming();
    }


    public boolean isOnTargetToKeepShooting() {
        return true;
    }

    private Loop mLoop = new Loop() {

        // Every time we transition states, we update the current state start
        // time and the state changed boolean (for one cycle)
        private double mWantStateChangeStartTime;

        @Override
        public void onStart(double timestamp) {
            synchronized (Superstructure.this) {
                mWantedState = WantedState.IDLE;
                mCurrentStateStartTime = timestamp;
                mWantStateChangeStartTime = timestamp;
                mLastDisturbanceShooterTime = timestamp;
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Superstructure.this) {
                SystemState newState = mSystemState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle(mStateChanged);
                    break;
                case RANGE_FINDING:
                    newState = handleRangeFinding();
                    break;
                default:
                    newState = SystemState.IDLE;
                }

                if (newState != mSystemState) {
                    System.out.println("Superstructure state " + mSystemState + " to " + newState + " Timestamp: "
                            + Timer.getFPGATimestamp());
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    private SystemState handleRangeFinding() {
        mLED.setWantedState(LED.WantedState.FIND_RANGE);

        switch (mWantedState) {
        case UNJAM:
            return SystemState.UNJAMMING;
        case UNJAM_SHOOT:
            return SystemState.UNJAMMING_WITH_SHOOT;
        case SHOOT:
            return SystemState.WAITING_FOR_ALIGNMENT;
        case MANUAL_FEED:
            return SystemState.JUST_FEED;
        case EXHAUST:
            return SystemState.EXHAUSTING;
        case HANG:
            return SystemState.HANGING;
        case RANGE_FINDING:
            return SystemState.RANGE_FINDING;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleIdle(boolean stateChanged) {
        if (stateChanged) {
            stop();
            mLED.setWantedState(LED.WantedState.OFF);
        }
        mCompressor.setClosedLoopControl(!mCompressorOverride);

        switch (mWantedState) {
        case UNJAM:
            return SystemState.UNJAMMING;
        case UNJAM_SHOOT:
            return SystemState.UNJAMMING_WITH_SHOOT;
        case SHOOT:
            return SystemState.WAITING_FOR_ALIGNMENT;
        case MANUAL_FEED:
            return SystemState.JUST_FEED;
        case EXHAUST:
            return SystemState.EXHAUSTING;
        case HANG:
            return SystemState.HANGING;
        case RANGE_FINDING:
            return SystemState.RANGE_FINDING;
        default:
            return SystemState.IDLE;
        }
    }

    private double getShootingSetpointRpm(double range) {
        if (Constants.kUseFlywheelAutoAimPolynomial) {
            return Constants.kFlywheelAutoAimPolynomial.predict(range);
        } else {
            return Constants.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }


    public synchronized double getCurrentTuningRpm() {
        return mCurrentTuningRpm;
    }

    public synchronized double getCurrentRange() {
        return mLastGoalRange;
    }

    public synchronized void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
    }

    public synchronized void setActuateHopper(boolean extended) {
        mHopperSolenoid.set(extended);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Air Pressure psi", mAirPressureSensor.getAirPressurePsi());
    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    public void setOverrideCompressor(boolean force_off) {
        mCompressorOverride = force_off;
    }

}
