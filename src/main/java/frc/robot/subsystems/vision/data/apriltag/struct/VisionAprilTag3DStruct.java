package frc.robot.subsystems.vision.data.apriltag.struct;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import frc.robot.subsystems.vision.data.apriltag.VisionAprilTag3D;

import java.nio.ByteBuffer;

 public class VisionAprilTag3DStruct implements Struct<VisionAprilTag3D> {
    @Override
    public Class<VisionAprilTag3D> getTypeClass() {
        return VisionAprilTag3D.class;
    }

    @Override
    public String getTypeName() {
        return "VisionAprilTag3D";
    }

    @Override
    public int getSize() {
        return kSizeInt32 + Pose2d.struct.getSize() + kSizeDouble * 2;
    }

    @Override
    public String getSchema() {
        return "Pose2d robotToTargetPose;int32 fiducialID;double ambiguity;double timestamp";
    }

    @Override
    public Struct<?>[] getNested() {
        return new Struct<?>[]{Pose2d.struct};
    }

    @Override
    public VisionAprilTag3D unpack(ByteBuffer bb) {
        VisionAprilTag3D visionAprilTag3D = new VisionAprilTag3D();
        unpackInto(visionAprilTag3D, bb);
        return visionAprilTag3D;
    }

    @Override
    public void unpackInto(VisionAprilTag3D out, ByteBuffer bb) {
        Pose2d robotToTargetPose = Pose2d.struct.unpack(bb);
        int fiducialID = bb.getInt();
        double ambiguity = bb.getDouble();
        double timestamp = bb.getDouble();

        out.setFrom(
                fiducialID, robotToTargetPose, ambiguity, timestamp
        );
    }

    @Override
    public void pack(ByteBuffer bb, VisionAprilTag3D value) {
        Pose2d.struct.pack(bb, value.robotToTargetPose());
        bb.putInt(value.fiducialID());
        bb.putDouble(value.ambiguity());
        bb.putDouble(value.timestamp());
    }
}
