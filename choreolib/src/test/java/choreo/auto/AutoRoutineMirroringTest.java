// Copyright (c) Choreo contributors

package choreo.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectoryTestHelper;
import choreo.util.ChoreoAllianceFlipUtil;
import choreo.util.ChoreoAllianceFlipUtil.Flipper;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

public class AutoRoutineMirroringTest {
  @Test
  void testMirrorXMirrorsTrackedTrajectoriesInPlace() {
    assert HAL.initialize(500, 0);
    ChoreoAllianceFlipUtil.setFlipper(Flipper.FRC_CURRENT);

    AutoFactory factory = AutoTestHelper.factory(false);
    AutoRoutine routine = factory.newRoutine("mirrorXRoutine");

    Pose2d firstStart = new Pose2d(1.0, 2.0, Rotation2d.fromRadians(0.4));
    Pose2d firstEnd = new Pose2d(3.0, 4.0, Rotation2d.fromRadians(0.8));
    Pose2d secondStart = new Pose2d(2.5, 1.5, Rotation2d.fromRadians(-0.6));
    Pose2d secondEnd = new Pose2d(5.0, 2.0, Rotation2d.fromRadians(1.1));
    Trajectory<SwerveSample> firstRawTrajectory =
        TrajectoryTestHelper.linearTrajectory("first", firstStart, firstEnd, 2.0, SwerveSample.class);
    Trajectory<SwerveSample> secondRawTrajectory =
        TrajectoryTestHelper.linearTrajectory(
            "second", secondStart, secondEnd, 2.0, SwerveSample.class);

    AutoTrajectory firstTrajectory = routine.trajectory(firstRawTrajectory);
    AutoTrajectory secondTrajectory = routine.trajectory(secondRawTrajectory);

    AutoRoutine mirroredRoutine = routine.mirrorX();

    assertSame(routine, mirroredRoutine);
    assertEquals(
        ChoreoAllianceFlipUtil.getMirrorX().flip(firstRawTrajectory.getInitialPose(false).get()),
        firstTrajectory.getInitialPose().get());
    assertEquals(
        ChoreoAllianceFlipUtil.getMirrorX().flip(firstRawTrajectory.getFinalPose(false).get()),
        firstTrajectory.getFinalPose().get());
    assertEquals(
        ChoreoAllianceFlipUtil.getMirrorX().flip(secondRawTrajectory.getInitialPose(false).get()),
        secondTrajectory.getInitialPose().get());
    assertEquals(
        ChoreoAllianceFlipUtil.getMirrorX().flip(secondRawTrajectory.getFinalPose(false).get()),
        secondTrajectory.getFinalPose().get());
  }

  @Test
  void testMirrorYMirrorsTrackedTrajectoriesInPlace() {
    assert HAL.initialize(500, 0);
    ChoreoAllianceFlipUtil.setFlipper(Flipper.FRC_CURRENT);

    AutoFactory factory = AutoTestHelper.factory(false);
    AutoRoutine routine = factory.newRoutine("mirrorYRoutine");

    Pose2d firstStart = new Pose2d(0.8, 1.2, Rotation2d.fromRadians(0.2));
    Pose2d firstEnd = new Pose2d(2.2, 5.1, Rotation2d.fromRadians(-1.0));
    Pose2d secondStart = new Pose2d(4.4, 0.9, Rotation2d.fromRadians(0.7));
    Pose2d secondEnd = new Pose2d(6.0, 3.6, Rotation2d.fromRadians(-0.3));
    Trajectory<SwerveSample> firstRawTrajectory =
        TrajectoryTestHelper.linearTrajectory(
            "firstY", firstStart, firstEnd, 2.0, SwerveSample.class);
    Trajectory<SwerveSample> secondRawTrajectory =
        TrajectoryTestHelper.linearTrajectory(
            "secondY", secondStart, secondEnd, 2.0, SwerveSample.class);

    AutoTrajectory firstTrajectory = routine.trajectory(firstRawTrajectory);
    AutoTrajectory secondTrajectory = routine.trajectory(secondRawTrajectory);

    AutoRoutine mirroredRoutine = routine.mirrorY();

    assertSame(routine, mirroredRoutine);
    assertEquals(
        ChoreoAllianceFlipUtil.getMirrorY().flip(firstRawTrajectory.getInitialPose(false).get()),
        firstTrajectory.getInitialPose().get());
    assertEquals(
        ChoreoAllianceFlipUtil.getMirrorY().flip(firstRawTrajectory.getFinalPose(false).get()),
        firstTrajectory.getFinalPose().get());
    assertEquals(
        ChoreoAllianceFlipUtil.getMirrorY().flip(secondRawTrajectory.getInitialPose(false).get()),
        secondTrajectory.getInitialPose().get());
    assertEquals(
        ChoreoAllianceFlipUtil.getMirrorY().flip(secondRawTrajectory.getFinalPose(false).get()),
        secondTrajectory.getFinalPose().get());
  }
}
