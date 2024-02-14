package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum AlignPosition {
            AmpPos(),
            SpeakerPos(),
            LeftSourcePos(),
            RightSourcePos(),
            ClimbPos();

            private static AlignPosition alignPosition;
            public static Pose2d alignPose;


            public static AlignPosition getPosition(){
                if(alignPose == null){
                    boolean isRed = Constants.isRedAlliance;
                    switch(alignPosition){
                        case AmpPos:
                            alignPose = isRed? new Pose2d(6.35, 4.1, Rotation2d.fromDegrees(90)): new Pose2d(-6.45, 4.1, Rotation2d.fromDegrees(90));
                            break;
                        case SpeakerPos:
                            alignPose = isRed? new Pose2d(8.3, 1.45, null): new Pose2d(-8.3, 1.45, null);
                            break;
                        case LeftSourcePos:
                            alignPose = isRed? new Pose2d(-7, -3.3, Rotation2d.fromDegrees(-145)): new Pose2d(7.8, -3.8, Rotation2d.fromDegrees(-55));
                            break;
                        case RightSourcePos:
                            alignPose = isRed? new Pose2d(-7.9, -3.8, Rotation2d.fromDegrees(-145)): new Pose2d(6.95, -3.3, Rotation2d.fromDegrees(-55));
                            break;
                        case ClimbPos:
                            alignPose = isRed? new Pose2d(3.4, 0, null): new Pose2d(-3.4, 0, null);
                            break;
                    }
                    return AlignPosition.SpeakerPos;
                }

                return alignPosition;
            }

            public static void setPosition(AlignPosition alignPos){
                alignPosition = alignPos;
                System.out.println("yup and you're banned and youre silly, and youre doxxed, and youre doing too much, and youre... yeah that's enough");
            }
                
            AlignPosition(){
                
            }
        }