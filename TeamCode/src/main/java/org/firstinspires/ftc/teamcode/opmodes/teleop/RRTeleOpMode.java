package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

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

    private Servo intake; //moving wheels
    private Servo wrist;
    private Servo drone;


    private double TURN_WRIST = 1; //turn it forward
    private double RESET_WRIST = 0.5; //so it doesn't swing 180 back

    double ServoPosition = 1;
    double ServoSpeed = 0.5;


    private double armMotorTicks = 5281.1;


    @Override
    public void runOpMode() throws InterruptedException {

        pixel = hardwareMap.get(TouchSensor.class, "pixel");
        intake = hardwareMap.get(Servo.class, "INTAKE");
        wrist = hardwareMap.get(Servo.class, "WRIST");
        drone = hardwareMap.get(Servo.class, "droneLauncher");
        DcMotor armMotor = hardwareMap.dcMotor.get("Arm");
        DcMotor liftMotor = hardwareMap.dcMotor.get("LIFT");

        // ARM Motor
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Lift Motor
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        double SLOW_DOWN_FACTOR = 0.75; //TODO Adjust to driver comfort
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
                    intake.setPosition(0); //stops the intake servos
                    telemetry.addData("Pixel", "Detected");
                    telemetry.update();
                }

                //Servo WRIST UP - this is working as of 12/30
                if (gamepad1.y) {
                    wrist.setDirection(Servo.Direction.REVERSE); //edit for only one signal bc of y cable
                    wrist.setPosition(TURN_WRIST); //edit for only one signal bc of y cable
                    ServoPosition += ServoSpeed;
                    telemetry.addData("Turn", "Over");
                    telemetry.update();
                }

                //Servo WRIST DOWN - this is working as of 12/30
                if (gamepad1.a) {
                    wrist.setDirection(Servo.Direction.REVERSE); //edit for only one signal bc of y cable
                    wrist.setPosition(RESET_WRIST); //edit for only one signal bc of y cable
                    ServoPosition += ServoSpeed;
                    telemetry.addData("Reset", "Servos");
                    telemetry.update();
                }
                //Drone Launcher -  this is working as of 12/30
                if(gamepad1.x){
                    drone.setDirection(Servo.Direction.REVERSE);
                    drone.setPosition(1);
                    telemetry.addData("Launching", "Drone");
                    telemetry.update();
                    sleep(1000);
                    drone.setPosition(0);
                }

                drive.updatePoseEstimate();

                if (gamepad1.right_bumper) {
                    intake.setDirection(Servo.Direction.REVERSE);
                    intake.setPosition(0.75);
                    sleep(300);
                }

                if (gamepad1.left_bumper) {
                    intake.setDirection(Servo.Direction.FORWARD);
                    intake.setPosition(0);
                }

                intake.setPosition(0);

                //Setup for RIGGING - takes ARM motor and LEFT motor at 45 deg angle and takes WRIST to UP position
                // Working as of 12/31
                if (gamepad1.b) {
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setTargetPosition(1100);
                    telemetry.addData("Up: ", armMotor.getCurrentPosition());
                    liftMotor.setTargetPosition(1394);
                    armMotor.setPower(1);
                    liftMotor.setPower(1);
                }

                //LIFT the whole robot for rigging
                // Working as of 12/31
                if (gamepad1.start) { //this resets the arm to attach the hook
                    armMotor.setTargetPosition(0);
                    liftMotor.setTargetPosition(0);
                    wrist.setPosition(0);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); //to be tested
                    armMotor.setPower(-0.75);
                }

                //ARM MOTOR - DPAD-UP goes UP, DPAD-DOWN goes DOWN
                //Working as of 12/30 and holds position
                //TODO Add guardrails for max and min values
                if (gamepad1.dpad_up) {
                    armMotor.setTargetPosition(armMotor.getCurrentPosition() + 50);
                    armMotor.setPower(0.75);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (gamepad1.dpad_down) {
                    armMotor.setTargetPosition(armMotor.getCurrentPosition() - 50);
                    armMotor.setPower(-0.75);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 }

                // LIFT MOTOR - DPAD LEFT goes DOWN, DPAD RIGHT goes UP
                //Working as of 12/31 and holds position
                if (gamepad1.dpad_right) {
                    liftMotor.setTargetPosition(liftMotor.getCurrentPosition() + 50);
                    liftMotor.setPower(0.3);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (gamepad1.dpad_left) {
                    liftMotor.setTargetPosition(liftMotor.getCurrentPosition() - 50);
                    liftMotor.setPower(-0.3);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                //ARM Motor telemetry
                telemetry.addData("-------------ARM-----------------","");
                telemetry.addData("Current Position for ARM ", armMotor.getCurrentPosition());
                telemetry.addData("Target Position for ARM", armMotor.getTargetPosition());

                //LIFT Motor telemetry
                telemetry.addData("---------------LIFT------------------------","");
                telemetry.addData("Current Position for LIFT ", liftMotor.getCurrentPosition());
                telemetry.addData("Target Position for LIFT", liftMotor.getTargetPosition());

                //DriveTrain
                telemetry.addData("-----------------DRIVE----------------------","");
                telemetry.addLine("Current Pose");
                telemetry.addData("Current X pos: ", drive.pose.position.x);
                telemetry.addData("Current Y pos: ", drive.pose.position.y);
                telemetry.addData("Current heading", Math.toDegrees(drive.pose.heading.log()));

                telemetry.update();
        }  //end of while

    } // end of opMode



}