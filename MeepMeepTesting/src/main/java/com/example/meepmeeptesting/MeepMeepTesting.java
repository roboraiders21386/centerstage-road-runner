package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d moveBeyondTrussPose = new Pose2d(0,0,0);
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0,0,0);
        Pose2d midwayPose1a = new Pose2d(0,0,0);
        Pose2d intakeStack = new Pose2d(0,0,0);
        Pose2d midwayPose2 = new Pose2d(0,0,0);
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0,0, 0);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 30, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(14,17.25)
                .build();

        dropPurplePixelPose = new Pose2d(26, 8, Math.toRadians(0));
        dropYellowPixelPose = new Pose2d(23, 36, Math.toRadians(-90));


        /*
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, -36, Math.toRadians(180)))
                .setReversed(true)
                //.afterTime(1,vision.switchFrontAction())
                .splineTo(new Vector2d(48,-36), Math.toRadians(0))
                .setReversed(false)
                .setTangent(Math.toRadians(180))
                //.afterTime(1,vision.switchBackAction())
                .splineTo(new Vector2d(-60,-36), Math.toRadians(180))

                .build());
*/
        myBot.runAction(myBot.getDrive().actionBuilder(initPose)
                        .splineTo(new Vector2d(48,-36), Math.toRadians(0))
                        .strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                        //.strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    // DUMMY ACTIONS
    public Action switchBackCam() {
        return telemetryPacket -> false;
    }
    public Action switchFrontCam() {
        return telemetryPacket -> false;
    }
}