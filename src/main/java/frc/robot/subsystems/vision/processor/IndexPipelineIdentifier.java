package frc.robot.subsystems.vision.processor;

public class IndexPipelineIdentifier implements PipelineIdentifier<Integer> {
    private final int index;

    private IndexPipelineIdentifier(int index) {
        this.index = index;
    }

    public static IndexPipelineIdentifier pipelineIndex(int index) {
        return new IndexPipelineIdentifier(index);
    }

    @Override
    public Integer getIdentifier() {
        return index;
    }
}
