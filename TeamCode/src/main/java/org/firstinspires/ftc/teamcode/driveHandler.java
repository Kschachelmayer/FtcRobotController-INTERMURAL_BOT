package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.teleOp1;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class driveHandler {

    private final LinearOpMode driveOpMode;

    public IMU imu = null;
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;
    public DcMotor spinner = null;
    public boolean fieldCentric = false;

    double FLpower = 0;
    double FRpower = 0;
    double BLpower = 0;
    double BRpower = 0;

    //IMU imu = driveOpMode.hardwareMap.get(IMU.class, "imu");

    public driveHandler(LinearOpMode opmode) {driveOpMode = opmode;}
    public void boot() {
        //front left and right, back left and right
        imu = driveOpMode.hardwareMap.get(IMU.class, "imu");
        FL = driveOpMode.hardwareMap.get(DcMotor.class, "FL");
        FR = driveOpMode.hardwareMap.get(DcMotor.class, "FR");
        BL = driveOpMode.hardwareMap.get(DcMotor.class, "BL");
        BR = driveOpMode.hardwareMap.get(DcMotor.class, "BR");
        spinner = driveOpMode.hardwareMap.get(DcMotor.class, "spinner");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        spinner.setDirection(DcMotorSimple.Direction.FORWARD);

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        spinner.setPower(0);


        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);


    }
    public void botDrive(double y,double x,double rx, double speed, boolean mode){
        //get bot heading
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //set values to account for speed
        x = x * speed;
        y = y * speed;
        rx = rx * speed;

        if(!mode){
            //robot centric mode
            FLpower = (y + x + rx);
            BLpower = (y - x + rx);
            FRpower = (y - x - rx);
            BRpower = (y + x - rx);
        }
        if(mode){
            //field centric mode
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            FLpower = (rotY + rotX + rx);
            BLpower = (rotY - rotX + rx);
            FRpower = (rotY - rotX - rx);
            BRpower = (rotY + rotX - rx);
        }

        //set motor powers
        FL.setPower(FLpower);
        FR.setPower(FRpower);
        BL.setPower(BLpower);
        BR.setPower(BRpower);
    }

    public void botControl(int modeNum){
        //handle actions outside of driving
        if(modeNum == 0){
            if(spinner.getPower() != 0){
                spinner.setPower(0);
            }else{
                spinner.setPower(0.5);
            }
        }
    }
}
