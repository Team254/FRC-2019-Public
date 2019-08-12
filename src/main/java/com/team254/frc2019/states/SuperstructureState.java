package com.team254.frc2019.states;

import com.team254.frc2019.Constants;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;

public class SuperstructureState {
    public double turret; // degrees
    public double elevator; // inches
    public double shoulder; // degrees
    public double wrist; // degrees

    public SuperstructureState(double turret, double elevator, double shoulder, double wrist) {
        this.turret = turret;
        this.elevator = elevator;
        this.shoulder = shoulder;
        this.wrist = wrist;
    }

    public SuperstructureState(SuperstructureState other) {
        this.turret = other.turret;
        this.elevator = other.elevator;
        this.shoulder = other.shoulder;
        this.wrist = other.wrist;
    }

    // default robot position
    public SuperstructureState() {
        this(0, 0, 0, 0);
    }

    public void setFrom(SuperstructureState source) {
        turret = source.turret;
        elevator = source.elevator;
        shoulder = source.shoulder;
        wrist = source.wrist;
    }

    /**
     * @return height of the bottom roller of the end effector; only applicable for SFR and SVR end effectors
     */
    public double getBottomEndEffectorHeight() {
        double z = Constants.kElevatorHeightToFloor; // z will represent the height of the bottom of the end effector to the ground in inches
        z += this.elevator;
        z += Constants.kArmLength * Math.sin(Math.toRadians(this.shoulder));
        z += Constants.kWristToBottomEndEffectorLength * Math.sin(Math.toRadians(this.wrist + Constants.kEndEffectorBottomAngle));
        return z;
    }

    /**
     * @return Translation2d where x maps to x position and y maps to z position
     */
    public Translation2d getPlanarWristJointLocation() {
        double z = this.elevator;
        z += Constants.kArmLength * Math.sin(Math.toRadians(this.shoulder));

        double x = Constants.kArmLength * Math.cos(Math.toRadians(this.shoulder));

        return new Translation2d(x, z);
    }

    /**
     * @return is bottom roller of the end effector above the bumper; only applicable for SFR and SVR end effectors
     */
    public boolean isOverBumper() {
        return getBottomEndEffectorHeight() > Constants.kBumperHeight + SuperstructureConstants.kElevatorPaddingInches;
    }

    public boolean isTurretSafeForWristBelowBumper() {
        return Util.epsilonEquals(turret, 0, SuperstructureConstants.kTurretPaddingDegrees)
                || Util.epsilonEquals(turret, 180, SuperstructureConstants.kTurretPaddingDegrees);
    }

    @Override
    public String toString() {
        return "SuperstructureState{" +
                "turret=" + turret +
                ", elevator=" + elevator +
                ", shoulder=" + shoulder +
                ", wrist=" + wrist +
                '}';
    }

    public Double[] asVector() {
        return new Double[]{turret, elevator, shoulder, wrist};
    }
}
