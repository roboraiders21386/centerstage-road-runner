package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
    public static class Params {
        //TODO Step 11.1 : Update values of par0YTicks, part1YTicks, perpXTicks from AngularRampLogger
        public double par0YTicks = -11400.633579319896; //-11702.617787599618; //0.0; // y position of the first parallel encoder (in tick units)
        public double par1YTicks = -11400.633579319896; //11636.31641072519; //1.0; // y position of the second parallel encoder (in tick units)
        public double perpXTicks = -11400.633579319896; //-9064.884736187201; //0.0; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    //public final Encoder par0, par1, perp;
    public final Encoder encoderLeft, encoderRight, encoderAux;


    public final double inPerTick;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;

    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick) {
        //TODO Step 3.1 : Update hardware configuration names for dead wheel encoders
        //par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par0")));
        //par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par1")));
        //perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "perp")));

        encoderLeft = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "LF")));
        encoderRight = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "RF")));
        encoderAux = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "LB")));

        encoderLeft.setDirection(DcMotorEx.Direction.REVERSE);

        //reserving the direction of the perpedicular encoder
        encoderAux.setDirection(DcMotorSimple.Direction.REVERSE);

        lastPar0Pos = encoderLeft.getPositionAndVelocity().position;
        lastPar1Pos = encoderRight.getPositionAndVelocity().position;
        lastPerpPos = encoderAux.getPositionAndVelocity().position;

        this.inPerTick = inPerTick;

        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair par0PosVel = encoderLeft.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = encoderRight.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = encoderAux.getPositionAndVelocity();

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        return twist;
    }
}
