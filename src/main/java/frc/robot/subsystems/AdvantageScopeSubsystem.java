package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AdvantageScopeSubsystem extends SubsystemBase{
    
    Pose2d poseA = new Pose2d();
    Pose2d poseB = new Pose2d();

    // WPILib
    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();
    StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

    public void periodic() {
        publisher.set(poseA);
        arrayPublisher.set(new Pose2d[] {poseA, poseB});
    }

}
