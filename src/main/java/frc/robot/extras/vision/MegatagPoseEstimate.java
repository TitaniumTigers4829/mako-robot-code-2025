package frc.robot.extras.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.extras.vision.TigerHelpers.PoseEstimate;
import java.nio.ByteBuffer;

/**
 * A struct representing a pose estimate from the Megatag system. While this basically just wraps a
 * {@link PoseEstimate}, it is useful for logging.
 */
public class MegatagPoseEstimate implements StructSerializable {
  public static class MegatagPoseEstimateStruct implements Struct<MegatagPoseEstimate> {
    public Pose2d fieldToCamera = Pose2d.kZero;
    public double timestampSeconds;
    public double latency;
    public double avgTagArea;
    public double avgTagDist;
    public int tagCount;

    @Override
    public Class<MegatagPoseEstimate> getTypeClass() {
      return MegatagPoseEstimate.class;
    }

    @Override
    public String getTypeString() {
      return "struct:MegatagPoseEstimate";
    }

    @Override
    public int getSize() {
      return Pose2d.struct.getSize() + kSizeDouble * 3;
    }

    @Override
    public String getSchema() {
      return "Pose2d fieldToCamera;double timestampSeconds;double latency;double avgTagArea; int tagCount; double avgTagDist";
    }

    @Override
    public Struct<?>[] getNested() {
      return new Struct<?>[] {Pose2d.struct};
    }

    @Override
    public MegatagPoseEstimate unpack(ByteBuffer bb) {
      MegatagPoseEstimate rv = new MegatagPoseEstimate();
      rv.fieldToCamera = Pose2d.struct.unpack(bb);
      rv.timestampSeconds = bb.getDouble();
      rv.latency = bb.getDouble();
      rv.avgTagArea = bb.getDouble();
      rv.fiducialIds = new int[0];
      rv.ambiguity = bb.getDouble();
      rv.avgTagDist = bb.getDouble();
      rv.timestampSeconds = bb.getDouble();
      return rv;
    }

    @Override
    public void pack(ByteBuffer bb, MegatagPoseEstimate value) {
      Pose2d.struct.pack(bb, value.fieldToCamera);
      bb.putDouble(value.timestampSeconds);
      bb.putDouble(value.latency);
      bb.putDouble(value.avgTagArea);
      bb.putDouble(value.avgTagDist);
    }

    @Override
    public String getTypeName() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'getTypeName'");
    }
  }

  public Pose2d fieldToCamera = Pose2d.kZero;
  public double timestampSeconds;
  public double latency;
  public double avgTagArea;
  public int[] fiducialIds;
  public double ambiguity;
  public int tagCount;
  public double avgTagDist;

  public MegatagPoseEstimate() {}

  public static MegatagPoseEstimate fromLimelight(PoseEstimate poseEstimate) {
    MegatagPoseEstimate rv = new MegatagPoseEstimate();
    rv.fieldToCamera = poseEstimate.pose;
    if (rv.fieldToCamera == null) rv.fieldToCamera = Pose2d.kZero;
    rv.timestampSeconds = poseEstimate.timestampSeconds;
    rv.latency = poseEstimate.latency;
    rv.avgTagArea = poseEstimate.avgTagArea;
    rv.avgTagDist = poseEstimate.avgTagDist;
    rv.tagCount = poseEstimate.tagCount;
    rv.fiducialIds = new int[poseEstimate.rawFiducials.length];
    rv.ambiguity = 0.0;
    for (int i = 0; i < rv.fiducialIds.length; ++i) {
      rv.fiducialIds[i] = poseEstimate.rawFiducials[i].id;
      rv.ambiguity = poseEstimate.rawFiducials[i].ambiguity;
    }

    return rv;
  }

  public static final MegatagPoseEstimateStruct struct = new MegatagPoseEstimateStruct();
}
