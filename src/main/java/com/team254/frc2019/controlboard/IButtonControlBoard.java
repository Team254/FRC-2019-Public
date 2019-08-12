package com.team254.frc2019.controlboard;

import com.team254.lib.geometry.Rotation2d;

public interface IButtonControlBoard {
    void reset();

    double getJogTurret();

    boolean getScorePresetLow();
    boolean getScorePresetMiddle();
    boolean getScorePresetHigh();
    boolean getScorePresetCargo();

    boolean getPresetStow();
    boolean getPickupDiskWall();
    boolean getPickupBallGround();

    void setRumble(boolean on);

    // Climbing
    boolean getToggleHangMode();
    boolean getToggleHangModeLow();
    double getElevatorThrottle();

    double getJoggingX();
    double getJoggingZ();

    // Turret
    enum TurretCardinal {
        BACK(180),
        FRONT(0),
        LEFT(90),
        RIGHT(-90),
        NONE(0),
        FRONT_LEFT(30, 45),
        FRONT_RIGHT(-30, -45),
        BACK_LEFT(150, 135),
        BACK_RIGHT(210, 235);

        public final Rotation2d rotation;
        private final Rotation2d inputDirection;

        TurretCardinal(double degrees) {
            this(degrees, degrees);
        }

        TurretCardinal(double degrees, double inputDirectionDegrees) {
            rotation = Rotation2d.fromDegrees(degrees);
            inputDirection = Rotation2d.fromDegrees(inputDirectionDegrees);
        }

        public static TurretCardinal findClosest(double xAxis, double yAxis) {
            return findClosest(new Rotation2d(yAxis, -xAxis, true));
        }

        public static TurretCardinal findClosest(Rotation2d stickDirection) {
            var values = TurretCardinal.values();

            TurretCardinal closest = null;
            double closestDistance = Double.MAX_VALUE;
            for (int i = 0; i < values.length; i++) {
                var checkDirection = values[i];
                var distance = Math.abs(stickDirection.distance(checkDirection.inputDirection));
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closest = checkDirection;
                }
            }
            return closest;
        }

        public static boolean isDiagonal(TurretCardinal cardinal) {
            return cardinal == FRONT_LEFT || cardinal == FRONT_RIGHT || cardinal == BACK_LEFT || cardinal == BACK_RIGHT;
        }
    }

    TurretCardinal getTurretCardinal();

    boolean getAutoAim();
}