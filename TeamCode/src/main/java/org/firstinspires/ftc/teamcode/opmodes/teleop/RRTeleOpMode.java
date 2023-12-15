package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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

        int armUpPosition = -300;
        int liftUpPosition = -150;
        int armDownPosition = 300;
        int liftDownPosition = 150;

        pixel = hardwareMap.get(TouchSensor.class, "pixel");
        INTAKE3 = hardwareMap.get(CRServo.class, "INTAKE3");
        INTAKE4 = hardwareMap.get(CRServo.class, "INTAKE4");
        wrist = hardwareMap.get(Servo.class, "WRIST");
        drone = hardwareMap.get(Servo.class, "droneLauncher");
        DcMotor armMotor = hardwareMap.dcMotor.get("Arm");
        DcMotor liftMotor = hardwareMap.dcMotor.get("LIFT");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(armDownPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(armDownPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double SLOW_DOWN_FACTOR = 0.5; //TODO Adjust to driver comfort
        telemetry.addData("Initializing TeleOp  for Team:", "21386");
        telemetry.update();

            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            drive.updatePoseEstimate();
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
                    drone.setDirection(Servo.Direction.FORWARD);
                    drone.setPosition(1);
                    telemetry.addData("Launching", "Drone");
                    telemetry.update();
                    sleep(1000);
                    drone.setPosition(0);
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

                if (gamepad1.b) {
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setTargetPosition(1100);
                    telemetry.addData("Up: ", armMotor.getCurrentPosition());
                    liftMotor.setTargetPosition(1394);
                    armMotor.setPower(1);
                    liftMotor.setPower(1);
                }
                if (gamepad1.start) { //this resets the arm to attach the hook
                    armMotor.setTargetPosition(0);
                    liftMotor.setTargetPosition(0);
                    wrist.setPosition(0);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); //to be tested
                    armMotor.setPower(-0.75);
                }

                if (gamepad1.dpad_up) {
                    armMotor.setTargetPosition(armMotor.getCurrentPosition()+50);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(1);
                    telemetry.addData("Up: ", armMotor.getCurrentPosition());
                }
                else if (gamepad1.dpad_down) {
                    armMotor.setTargetPosition(armMotor.getCurrentPosition()-50);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(-0.75);
                    telemetry.addData("Down: ", armMotor.getCurrentPosition());
                } else {armMotor.setPower(0);}
                if (gamepad1.dpad_left) {
                    liftMotor.setTargetPosition(armMotor.getCurrentPosition()+50);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftMotor.setPower(1);
                } else if (gamepad1.dpad_right) {
                    liftMotor.setTargetPosition(armMotor.getCurrentPosition()-50);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftMotor.setPower(-1);
                }

                double position = armMotor.getCurrentPosition();
                double position2 = liftMotor.getCurrentPosition();
                double desiredPosition = armMotor.getTargetPosition();
                double desiredPosition2 = liftMotor.getTargetPosition();
                telemetry.addData("Encoder Position", position);
                telemetry.addData("Encoder Position", position2);
                telemetry.addData("Desired Position", desiredPosition);
                telemetry.addData("Desired Position", desiredPosition2);
                telemetry.update();

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