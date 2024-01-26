package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * Robo Raiders TeleOp Testing
 *
 */

@TeleOp(name = "RR TeleOp Dup", group = "00-Teleop")
public class RRTeleOpModeDup extends LinearOpMode {

    //private TouchSensor pixel;

    private Servo intake; //Intake/Claw AXON Servo
    private Servo wrist;
    private Servo drone;

    // WRIST related parameters
    private double TURN_WRIST = 0.5; //turn it forward
    private double RESET_WRIST = 0.2; //so it doesn't swing 180 back

    private double WRIST_SERVO_MAX = 0.8; //1; //0.9;
    private double WRIST_SERVO_MIN = 0.2; //0.5;
    ElapsedTime wristServoTimer = new ElapsedTime();
    final double SERVOWAIT = 500; //20;
    final double SERVOINC = 0.05;
    double targetServoPosition = 0;

    private int TRIGGERTIME = 500;

    double wristServoPosition = 0.0; //Incremental servo position for the wrist
    double ServoPosition = 1;
    double ServoSpeed = 0.5;

    //CLAW parameters
    //RELEASE -> OPEN
    //GRAB -> CLOSE
    final double CLAW_RELEASE = 1;
    final double CLAW_GRAB = 0;

    private double armMotorTicks = 5281.1;

    ColorSensor colorSensor;    // Hardware Device Object
    LED RLED, LLED;



    @Override
    public void runOpMode() throws InterruptedException {

        //pixel = hardwareMap.get(TouchSensor.class, "pixel");
        intake = hardwareMap.get(Servo.class, "INTAKE");
        wrist = hardwareMap.get(Servo.class, "WRIST");
        drone = hardwareMap.get(Servo.class, "droneLauncher");
        DcMotor armMotor = hardwareMap.dcMotor.get("Arm");
        DcMotor liftMotor = hardwareMap.dcMotor.get("LIFT");
        //Color Sensor and LED
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        RLED = hardwareMap.get(LED.class, "RLED");
        LLED = hardwareMap.get(LED.class, "LLED");

        // ARM Motor
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Lift Motor
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        double SLOW_DOWN_FACTOR = 1; //TODO Adjust to driver comfort
        telemetry.addData("Initializing TeleOp  for Team:", "21386");
        telemetry.update();

        //Switch off LEDs
        RLED.enableLight(false); LLED.enableLight(false);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        drive.updatePoseEstimate();
        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("Running TeleOp Mode adopted for Team:", "21386");

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                            gamepad1.left_stick_x * SLOW_DOWN_FACTOR
                    ),
                    -gamepad1.right_stick_x * SLOW_DOWN_FACTOR
            ));


/*
                if (pixel.isPressed()) {
                    intake.setPosition(0); //stops the intake servos
                    telemetry.addData("Pixel", "Detected");
                    telemetry.update();
                }

 */
            if ((colorSensor.red() > 100)|| (colorSensor.green() >100) || (colorSensor.blue() > 100)){
                RLED.enableLight(true);  LLED.enableLight(true);
            }else{
                RLED.enableLight(false); LLED.enableLight(false);
            }


            //Servo WRIST UP - this is working as of 12/30
            if (gamepad1.y && !gamepad1.back && wristServoTimer.milliseconds() > SERVOWAIT) {
                wrist.setDirection(Servo.Direction.REVERSE);
                wristServoTimer.reset();
                targetServoPosition = TURN_WRIST;
                telemetry.addData("Turn", "Over");
                telemetry.update();
            }

            //Servo WRIST DOWN - this is working as of 12/30
            if (gamepad1.a && !gamepad1.back && wristServoTimer.milliseconds() > SERVOWAIT) {
                wrist.setDirection(Servo.Direction.REVERSE); //edit for only one signal bc of y cable
                wristServoTimer.reset();
                targetServoPosition = RESET_WRIST;
                telemetry.addData("Reset", "Servos");
                telemetry.update();
            }

            //Move Servo Wrist Incrementally - Raising
            //Between WRIST_SERVO_MIN, WRIST_SERVO_MAX
            if (gamepad1.right_trigger > 0 && wristServoTimer.milliseconds() > SERVOWAIT) {
                wristServoTimer.reset();
                targetServoPosition = wrist.getPosition() +  SERVOINC;
            }

            //Move Servo Wrist Incrementally - Lowering
            //Between WRIST_SERVO_MIN, WRIST_SERVO_MAX
            if (gamepad1.left_trigger > 0 && wristServoTimer.milliseconds() > SERVOWAIT ) {
                wristServoTimer.reset();
                targetServoPosition = wrist.getPosition() -  SERVOINC;
            }

            //Set the WRIST position along with the guardrails
            wrist.setPosition(Range.clip(targetServoPosition, WRIST_SERVO_MIN, WRIST_SERVO_MAX));


            //Drone Launcher -  this is working as of 12/30
            if(gamepad1.x && !gamepad1.back){
                drone.setDirection(Servo.Direction.REVERSE);
                drone.setPosition(1);
                telemetry.addData("Launching", "Drone");
                telemetry.update();
                sleep(1000);
                drone.setPosition(0);
            }
            if (gamepad1.y && gamepad1.back) {
                wrist.setPosition(TURN_WRIST);
                armMotor.setTargetPosition(219);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("Up: ", armMotor.getCurrentPosition());
                armMotor.setPower(1);
            }
            if (gamepad1.b && gamepad1.back) {
                wrist.setPosition(0.45);

                liftMotor.setTargetPosition(360);//(4000);//(1394);
                armMotor.setTargetPosition(453);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("Up: ", armMotor.getCurrentPosition());
                liftMotor.setPower(1);
                armMotor.setPower(1);
            }
            if (gamepad1.a && gamepad1.back) {
                wrist.setPosition(TURN_WRIST);
                liftMotor.setTargetPosition(718);//(4000);//(1394);
                armMotor.setTargetPosition(955);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("Up: ", armMotor.getCurrentPosition());
                liftMotor.setPower(1);
                armMotor.setPower(1);
                wrist.setPosition(TURN_WRIST);
            }
            if (gamepad1.x && gamepad1.back) {
                wrist.setPosition(0.45);
                liftMotor.setTargetPosition(995);//(4000);//(1394);
                armMotor.setTargetPosition(1014);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("Up: ", armMotor.getCurrentPosition());
                liftMotor.setPower(1);
                armMotor.setPower(1);
                wrist.setPosition(TURN_WRIST);
            }

            drive.updatePoseEstimate();

            //TODO Claw not working as of 1/1/2024 - need to troubleshoot
            // INTAKE - Grab CLAW Closing
            if (gamepad1.right_bumper) {
                intake.setDirection(Servo.Direction.REVERSE);
                intake.setPosition(CLAW_GRAB); // made it 1 on 1/1/2024
                sleep(300);
            }
            // INTAKE - Release CLAW Opening
            if (gamepad1.left_bumper) {
                intake.setDirection(Servo.Direction.REVERSE);
                intake.setPosition(CLAW_RELEASE); // made it 1 on 1/1/2024
            }
            //intake.setPosition(0);  //TODO - find out why was this setup here

            //Setup for RIGGING - takes ARM motor and LEFT motor at 45 deg angle and takes WRIST to UP position
            // Working as of 12/31
            if (gamepad1.b && !gamepad1.back) {
                liftMotor.setTargetPosition(1500);//(4000);//(1394);
                armMotor.setTargetPosition(1100);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("Up: ", armMotor.getCurrentPosition());
                liftMotor.setPower(1);
                armMotor.setPower(1);
                wrist.setPosition(TURN_WRIST);
            }

            //LIFT the whole robot for rigging
            // Working as of 12/31
            if (gamepad1.start) { //this resets the arm to attach the hook
                armMotor.setTargetPosition(0);
                liftMotor.setTargetPosition(0);
                //wrist.setPosition(0);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); //to be tested
                sleep(300);
                wrist.setPosition(RESET_WRIST);
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

            //Wrist Servo position
            telemetry.addData("-----------------WRIST SERVO----------------------","");
            telemetry.addData("Current Wrist servo pos: ", wrist.getPosition());
            telemetry.addData("Target Wrist servo pos: ", targetServoPosition);


            telemetry.update();
        }  //end of while

    } // end of opMode



}