package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public final class drive {
    private LinearOpMode teleOp1 = null;

    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;

    private Servo piston = null;

    public drive (LinearOpMode opmode) {
        teleOp1 = opmode;
    }
    public void init() {
        //front left and right, back left and right
        FL = teleOp1.hardwareMap.get(DcMotor.class, "FL");
        FR = teleOp1.hardwareMap.get(DcMotor.class, "FR");
        BL = teleOp1.hardwareMap.get(DcMotor.class, "BL");
        BR = teleOp1.hardwareMap.get(DcMotor.class, "BR");
        piston = teleOp1.hardwareMap.get(Servo.class, "piston");

        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        piston.setPosition(0);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU imu = HardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }
    public void botDrive(double y,double x,double rx){
        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double FLpower = (y + x + rx) / denom;
        double FRpower = (y - x - rx) / denom;
        double BLpower = (y - x + rx) / denom;
        double BRpower = (y + x - rx) / denom;
    }
}
