package frc.robot.subsystems.vision.data.nn;

public class VisionNNID<T> {
    private final T[] objIds;

    public VisionNNID(T[] objIds) {
        this.objIds = objIds;
    }

    public static <E extends Enum<E>> VisionNNID<E> fromEnumValues(E[] enumValues) {
      return new VisionNNID<>(enumValues);
    }

    private T getObjectIdentifier(int idx) {
        if (idx < 0 || idx >= objIds.length) return null;

        return objIds[idx];
    }

    public T getObjectIdentifier(VisionNNDetection detection) {
        return getObjectIdentifier(detection.classID());
    }
}
