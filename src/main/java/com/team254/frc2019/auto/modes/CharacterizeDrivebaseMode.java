package com.team254.frc2019.auto.modes;

import com.team254.frc2019.auto.AutoModeEndedException;
import com.team254.frc2019.auto.actions.CollectAccelerationDataAction;
import com.team254.frc2019.auto.actions.CollectVelocityDataAction;
import com.team254.frc2019.auto.actions.WaitAction;
import com.team254.lib.physics.DriveCharacterization;

import java.util.ArrayList;
import java.util.List;

public class CharacterizeDrivebaseMode extends AutoModeBase {
    private final boolean highGear;
    private final boolean reverse;
    private final boolean turn;

    public CharacterizeDrivebaseMode(boolean highGear, boolean reverse, boolean turn) {
        this.highGear = highGear;
        this.reverse = reverse;
        this.turn = turn;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        List<DriveCharacterization.DataPoint> velocityData = new ArrayList<>();
        List<DriveCharacterization.DataPoint> accelerationData = new ArrayList<>();

        runAction(new CollectVelocityDataAction(velocityData, highGear, reverse, turn));
        runAction(new WaitAction(10));
        runAction(new CollectAccelerationDataAction(accelerationData, highGear, reverse, turn));

        DriveCharacterization.CharacterizationConstants constants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);

        System.out.println("ks: " + constants.ks);
        System.out.println("kv: " + constants.kv);
        System.out.println("ka: " + constants.ka);
    }
}