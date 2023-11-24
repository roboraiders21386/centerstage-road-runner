package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Robo Raiders TeleOp Testing
 *
 */

@TeleOp(name = "RR TeleOp", group = "00-Teleop")
public class RRTeleOpMode extends LinearOpMode {

    private TouchSensor pixel;

    private CRServo INTAKE3; //moving wheels
    private CRServo INTAKE4;

    private Servo wrist;
    private Servo drone;


    private double TURN_WRIST = 1; //turn it forward
    private double RESET_WRIST = 0.5; //so it doesn't swing 180 back

    double ServoPosition = 1;
    double ServoSpeed = 0.5;

    private Servo RWRIST;
    private Servo LWRIST;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean IntakeGrab;
        boolean IntakeRelease;
        boolean LowerLeft;
        boolean RaiseRight;

        pixel = hardwareMap.get(TouchSensor.class, "pixel");
        INTAKE3 = hardwareMap.get(CRServo.class, "INTAKE3");
        INTAKE4 = hardwareMap.get(CRServo.class, "INTAKE4");
        wrist = hardwareMap.get(Servo.class, "WRIST");
        drone = hardwareMap.get(Servo.class, "droneLauncher");

        double SLOW_DOWN_FACTOR = 0.5; //TODO Adjust to driver comfort
        telemetry.addData("Initializing TeleOp  for Team:", "21386");
        telemetry.update();

            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                telemetry.addData("Running TeleOp Mode adopted for Team:", "21386");
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                                -gamepad1.left_stick_x * SLOW_DOWN_FACTOR
                        ),
                        -gamepad1.right_stick_x * SLOW_DOWN_FACTOR
                ));

                if (pixel.isPressed()) {
                    INTAKE3.setPower(0); //stops the intake servos
                    INTAKE4.setPower(0);
                    telemetry.addData("Pixel", "Detected");
                    telemetry.update();
                }

                if (gamepad1.y) {
                    wrist.setDirection(Servo.Direction.REVERSE); //edit for only one signal bc of y cable
                    wrist.setPosition(TURN_WRIST); //edit for only one signal bc of y cable
                    ServoPosition += ServoSpeed;
                    telemetry.addData("Turn", "Over");
                    telemetry.update();
                }

                if (gamepad1.a) {
                    wrist.setDirection(Servo.Direction.REVERSE); //edit for only one signal bc of y cable
                    wrist.setPosition(RESET_WRIST); //edit for only one signal bc of y cable
                    ServoPosition += ServoSpeed;
                    telemetry.addData("Reset", "Servos");
                    telemetry.update();
                }

                if(gamepad1.x){
                    drone.setDirection(Servo.Direction.REVERSE);
                    drone.setPosition(1);
                    telemetry.addData("Launching", "Drone");
                    telemetry.update();
                }

                drive.updatePoseEstimate();

                if (gamepad1.right_bumper) {
                    INTAKE3.setDirection(CRServo.Direction.REVERSE);
                    INTAKE4.setDirection(CRServo.Direction.FORWARD);
                    INTAKE4.setPower(0.75);
                    INTAKE3.setPower(0.75);
                    sleep(300);
                }

                if (gamepad1.left_bumper) {
                    INTAKE3.setDirection(CRServo.Direction.FORWARD);
                    INTAKE4.setDirection(CRServo.Direction.REVERSE);

                    if (pixel.isPressed()) {
                        INTAKE3.setPower(0); //stops the intake servos
                        INTAKE4.setPower(0);
                        telemetry.addData("Pixel", "Detected");
                        telemetry.update();
                    } else {
                        INTAKE4.setPower(0.75);
                        INTAKE3.setPower(0.75);
                        telemetry.addData("Intake", "Running!");
                        telemetry.update();
                        sleep(300);
                    }
                }

                INTAKE3.setPower(0);
                INTAKE4.setPower(0);


                //telemetry.addData("LF Encoder", drive.leftFront.getCurrentPosition());
                //telemetry.addData("LB Encoder", drive.leftBack.getCurrentPosition());
                //telemetry.addData("RF Encoder", drive.rightFront.getCurrentPosition());
                //telemetry.addData("RB Encoder", drive.rightBack.getCurrentPosition());

                telemetry.addLine("Current Pose");
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", Math.toDegrees(drive.pose.heading.log()));
                telemetry.update();

            //telemetry.addData("x", drive.pose.position.x);
            //telemetry.addData("y", drive.pose.position.y);
            //telemetry.addData("heading", drive.pose.heading);
            telemetry.update();
        }
    }
}