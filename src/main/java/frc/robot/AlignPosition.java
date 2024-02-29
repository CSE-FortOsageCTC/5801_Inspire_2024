package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum AlignPosition {
            AmpPos(),
            SpeakerPos(),
            SourcePos(),
            ClimbPos(),
            AutoPickup(),
            Manual(), 
            TrapPos1(),
            TrapPos2(),
            TrapPos3();

            private static AlignPosition alignPosition;
            private static Pose2d alignPose;

            public static AlignPosition getPosition(){

                if (alignPosition == null) {
                    alignPosition = SpeakerPos;
                }

                if(alignPose == null){
                    boolean isRed = Constants.isRedAlliance;
                    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
                    //layout.setOrigin(new Pose3d(0,0,0, new Rotation3d()));
                    switch(alignPosition){
                        case AmpPos:
                            alignPose = isRed? new Pose2d(6.35, 4.1, Rotation2d.fromDegrees(90)): new Pose2d(-6.45, 4.1, Rotation2d.fromDegrees(90));
                            break;
                        case SpeakerPos:
                            alignPose = isRed? layout.getTagPose(4).get().toPose2d() : layout.getTagPose(7).get().toPose2d();
                            break;
                        case SourcePos:
                            alignPose = isRed? new Pose2d(-7, -3.3, Rotation2d.fromDegrees(-145)): new Pose2d(7.8, -3.8, Rotation2d.fromDegrees(-55));
                            break;
                        case ClimbPos:
                            alignPose = isRed? new Pose2d(3.4, 0, null): new Pose2d(-3.4, 0, null);
                            break;
                        case AutoPickup:
                            alignPose = null;
                            break;
                        case Manual:
                            alignPose = null;
                            break;
                        case TrapPos1:
                            alignPose = isRed? new Pose2d(3.85, -0.85, Rotation2d.fromDegrees(240)): new Pose2d(-3.9, 0.78,  Rotation2d.fromDegrees(60));
                            break;
                        case TrapPos2:
                            alignPose = isRed? new Pose2d(3.85, 0.78, Rotation2d.fromDegrees(120)): new Pose2d(-3.9, -0.85, Rotation2d.fromDegrees(300));
                            break;
                        case TrapPos3:
                            alignPose = isRed? new Pose2d(2.45, 0, Rotation2d.fromDegrees(0)): new Pose2d(-2.45, 0, Rotation2d.fromDegrees(180));
                            break;
                    }
                }

                

                return alignPosition;
            }

            public static Pose2d getAlignPose() {
                return alignPose;
            }

            public static void setPosition(AlignPosition alignPos){
                alignPosition = alignPos;
                AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

                boolean isRed = Constants.isRedAlliance;
                    switch(alignPosition){
                        case AmpPos:
                            alignPose = isRed? new Pose2d(6.35, 4.1, Rotation2d.fromDegrees(90)): new Pose2d(-6.45, 4.1, Rotation2d.fromDegrees(90));
                            break;
                        case SpeakerPos:
                            alignPose = isRed? new Pose2d(8.3, 1.45, null): new Pose2d(-8.3, 1.45, null);
                            //alignPose = isRed? layout.getTagPose(4).get().toPose2d() : layout.getTagPose(7).get().toPose2d();
                            break;
                        case SourcePos:
                            alignPose = isRed? new Pose2d(-7, -3.3, Rotation2d.fromDegrees(-145)): new Pose2d(7.8, -3.8, Rotation2d.fromDegrees(-55));
                            break;
                        case ClimbPos:
                            alignPose = isRed? new Pose2d(3.4, 0, null): new Pose2d(-3.4, 0, null);
                            break;
                        case AutoPickup:
                            alignPose = null;
                            break;
                        case Manual:
                            alignPose = null;
                            break;
                        case TrapPos1:
                            alignPose = isRed? new Pose2d(3.85, -0.85, Rotation2d.fromDegrees(240)): new Pose2d(-3.9, 0.78,  Rotation2d.fromDegrees(60));
                            break;
                        case TrapPos2:
                            alignPose = isRed? new Pose2d(3.85, 0.78, Rotation2d.fromDegrees(120)): new Pose2d(-3.9, -0.85, Rotation2d.fromDegrees(300));
                            break;
                        case TrapPos3:
                            alignPose = isRed? new Pose2d(2.45, 0, Rotation2d.fromDegrees(0)): new Pose2d(-2.45, 0, Rotation2d.fromDegrees(180));
                            break;
                    }
            }
                
            AlignPosition(){
                
            }
        }