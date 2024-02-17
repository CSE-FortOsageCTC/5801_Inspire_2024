package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum AlignPosition {
            AmpPos(),
            SpeakerPos(),
            LeftSourcePos(),
            RightSourcePos(),
            ClimbPos(),
            AutoPickup(),
            Manual();

            private static AlignPosition alignPosition;
            private static Pose2d alignPose;

            public static AlignPosition getPosition(){
                
                if (alignPosition == null) {
                    alignPosition = SpeakerPos;
                }

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
                        case AutoPickup:
                            alignPose = null;
                            break;
                        case Manual:
                            alignPose = null;
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
                System.out.println("yup and you're banned and youre silly, and youre doxxed, and youre doing too much, and youre... yeah that's enough. Well no its not enough because anyone who has seen this code knows that we can't write two lines of code because NOOOO that would be 2 ez. We have to account for every flippin variable on the field and that everything has to be perfect. and then we have to run it by brandon and then then then... error 404 couldn't find the rest of this rant");
            }
                
            AlignPosition(){
                
            }
        }