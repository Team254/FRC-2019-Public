package com.team254.frc2019.auto.actions;

import com.team254.frc2019.RobotState;
import com.team254.lib.geometry.Translation2d;

public class WaitUntilInsideRegion implements Action {
    private final static RobotState mRobotState = RobotState.getInstance();

    private final Translation2d mBottomLeft;
    private final Translation2d mTopRight;

    public WaitUntilInsideRegion(Translation2d bottomLeft, Translation2d topRight) {
        mBottomLeft = bottomLeft;
        mTopRight = topRight;
    }

    @Override
    public boolean isFinished() {
        Translation2d position = mRobotState.getLatestFieldToVehicle().getValue().getTranslation();
        return position.x() > mBottomLeft.x() && position.x() < mTopRight.x()
                && position.y() > mBottomLeft.y() && position.y() < mTopRight.y();
    }

    @Override
    public void update() {}

    @Override
    public void done() {}

    @Override
    public void start() {}
}
