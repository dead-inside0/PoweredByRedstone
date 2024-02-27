package org.firstinspires.ftc.teamcode.RoadRunnerGuide.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import org.firstinspires.ftc.teamcode.RoadRunnerGuide.trajectorysequence.sequencesegment.SequenceSegment;

import java.util.List;

public final class WaitSegment extends SequenceSegment {
    public WaitSegment(Pose2d pose, double seconds, List<TrajectoryMarker> markers) {
        super(seconds, pose, pose, markers);
    }
}
