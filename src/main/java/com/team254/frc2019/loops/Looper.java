package com.team254.frc2019.loops;

import com.team254.frc2019.Constants;
import com.team254.lib.util.CrashTrackingRunnable;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

/**
 * This code runs all of the robot's loops. Loop objects are stored in a List object. They are started when the robot
 * powers up and stopped after the match.
 */
public class Looper implements ILooper {
    public final double kPeriod = Constants.kLooperDt;

    private boolean mRunning;

    private final Notifier mNotifier;
    private final List<Loop> mLoops;
    private final Object mTaskRunningLock = new Object();
    private double mTimestamp = 0;
    private double mDT = 0;

    private final CrashTrackingRunnable runnable_ = new CrashTrackingRunnable() {
        @Override
        public void runCrashTracked() {
            synchronized (mTaskRunningLock) {
                if (mRunning) {
                    double now = Timer.getFPGATimestamp();

                    for (Loop loop : mLoops) {
                        loop.onLoop(now);
                    }

                    mDT = now - mTimestamp;
                    mTimestamp = now;
                }
            }
        }
    };

    public Looper() {
        mNotifier = new Notifier(runnable_);
        mRunning = false;
        mLoops = new ArrayList<>();
    }

    @Override
    public synchronized void register(Loop loop) {
        synchronized (mTaskRunningLock) {
            mLoops.add(loop);
        }
    }

    public synchronized void start() {
        if (!mRunning) {
            System.out.println("Starting loops");

            synchronized (mTaskRunningLock) {
                mTimestamp = Timer.getFPGATimestamp();
                for (Loop loop : mLoops) {
                    loop.onStart(mTimestamp);
                }
                mRunning = true;
            }

            mNotifier.startPeriodic(kPeriod);
        }
    }

    public synchronized void stop() {
        if (mRunning) {
            System.out.println("Stopping loops");
            mNotifier.stop();

            synchronized (mTaskRunningLock) {
                mRunning = false;
                mTimestamp = Timer.getFPGATimestamp();
                for (Loop loop : mLoops) {
                    System.out.println("Stopping " + loop);
                    loop.onStop(mTimestamp);
                }
            }
        }
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("looper_dt", mDT);
    }
}
