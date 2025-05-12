# Vision Handle System

This document provides an overview of the vision handle system and how to use it to create and manage vision processors for different camera types.

## Overview

The vision handle system allows you to create and manage vision processors for different camera types (Limelight or PhotonVision) based on what the user configures. A vision handle doesn't need to have all types of vision processors, just the ones the user configures based on what type of implementation it is.

## Key Components

- **VisionHandle**: Represents a camera with one or more vision processors
- **VisionHandleFactory**: Creates vision handles with configurable processors
- **VisionSubsystem**: Manages vision handles and provides convenience methods

## Usage Examples

### Creating a Vision Handle with AprilTag Processing

```java
// In your robot initialization code
VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrain);

// Create a vision handle for a predefined camera ID with AprilTag processing
Transform3d robotToCameraTransform = new Transform3d(/* ... */);
VisionAprilTagSettings.VisionAprilTagMode aprilTagMode = 
    new VisionAprilTagSettings.VisionAprilTagMode(
        VisionAprilTagSettings.VisionAprilTagOptions.ALL_DETECTIONS);

VisionHandle limelightHandle = visionSubsystem.createAprilTagVisionHandle(
    VisionCameraID.BELUGA_LIMELIGHT,
    robotToCameraTransform,
    aprilTagMode);

// Later, get the AprilTag processor and use it
visionSubsystem.getAprilTagProcessor(VisionCameraID.BELUGA_LIMELIGHT).ifPresent(processor -> {
    List<VisionRobotPose> poses = processor.getRobotPoseObservation();
    // Process the poses...
});

// Alternatively, use the generic getProcessor method
visionSubsystem.getProcessor(VisionCameraID.BELUGA_LIMELIGHT, VisionProcessorType.APRILTAG).ifPresent(processor -> {
    List<VisionRobotPose> poses = processor.getRobotPoseObservation();
    // Process the poses...
});
```

### Creating a Vision Handle with Neural Network Processing

```java
// Create a vision handle for a predefined camera ID with neural network processing
String[] classNames = {"Cube", "Cone", "Note"};
VisionHandle photonHandle = visionSubsystem.createNNVisionHandle(
    VisionCameraID.RADIO_CAM,
    robotToCameraTransform,
    classNames);

// Later, get the neural network processor and use it
visionSubsystem.getNNProcessor(VisionCameraID.RADIO_CAM).ifPresent(processor -> {
    List<VisionNNDetection> detections = processor.getLatestNNDetections();
    // Process the detections...
});

// Alternatively, use the generic getProcessor method
visionSubsystem.getProcessor(VisionCameraID.RADIO_CAM, VisionProcessorType.NN).ifPresent(processor -> {
    List<VisionNNDetection> detections = processor.getLatestNNDetections();
    // Process the detections...
});
```

### Creating a Vision Handle with Multiple Processors

```java
// Create a vision handle with both AprilTag and neural network processing using the builder pattern
AbstractVisionHandleBuilder builder = visionSubsystem.createVisionHandleBuilder(
    VisionCameraID.SCORE_CAM,
    robotToCameraTransform);

// Set pipeline indices for processors (must be done before adding processors)
builder.setPipelineIndex(VisionProcessorType.APRILTAG, 0)
       .setPipelineIndex(VisionProcessorType.NN, 1);

// Add processors
builder.addAprilTagProcessor(aprilTagMode)
       .addNNProcessor(classNames);

// Build and register the handle
VisionHandle multiHandle = visionSubsystem.createVisionHandle(builder);

// Switch between processor types
// This will also switch the camera pipeline to the one associated with the processor
if (useAprilTags) {
    multiHandle.setCurrentProcessor(VisionProcessorType.APRILTAG);
} else {
    multiHandle.setCurrentProcessor(VisionProcessorType.NN);
}

// Get the active processor
VisionProcessor activeProcessor = multiHandle.activeVisionProcessor();
```

### Creating a Vision Handle with Custom Processors

```java
// Create a vision handle with custom processors using the builder pattern
VisionHandleBuilder builder = visionSubsystem.createVisionHandleBuilder(
    "customcam",
    VisionCameraID.VisionType.PHOTONVISION,
    robotToCameraTransform);

// Add standard processors
builder.addAprilTagProcessor(aprilTagMode);

// Add a custom processor (assuming you've created a custom processor type)
MyCustomProcessor customProcessor = new MyCustomProcessor();
builder.addProcessor(MyCustomProcessorType.CUSTOM, customProcessor);

// Build and register the handle
VisionHandle customHandle = visionSubsystem.createVisionHandle(builder);
```

## Advanced Usage

### Direct Access to Vision Handles

```java
// Get a vision handle by name
Optional<VisionHandle> handle = visionSubsystem.getVisionHandle("camera1");

// Check if a handle has a specific processor type
handle.ifPresent(h -> {
    if (h.hasProcessor(VisionProcessorType.APRILTAG)) {
        // Use the AprilTag processor
        VisionAprilTagProcessor aprilTagProcessor = h.getProcessor(VisionProcessorType.APRILTAG);
        // ...
    }
});
```

### Using Predefined Camera IDs

```java
// Get a vision handle for a predefined camera ID
Optional<VisionHandle> handle = visionSubsystem.getVisionHandle(VisionCameraID.RADIO_CAM);

// Get an AprilTag processor for a predefined camera ID
Optional<VisionAprilTagProcessor> processor = 
    visionSubsystem.getAprilTagProcessor(VisionCameraID.BELUGA_LIMELIGHT);

// Get a processor of any type for a predefined camera ID using the generic method
Optional<VisionNNProcessor> nnProcessor = 
    visionSubsystem.getProcessor(VisionCameraID.RADIO_CAM, VisionProcessorType.NN);

// Create a vision handle builder for a predefined camera ID
AbstractVisionHandleBuilder builder = visionSubsystem.createVisionHandleBuilder(
    VisionCameraID.SCORE_CAM,
    robotToCameraTransform);
```
