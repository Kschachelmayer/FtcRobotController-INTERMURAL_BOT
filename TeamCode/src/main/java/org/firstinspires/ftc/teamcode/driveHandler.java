package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.teleOp1;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class driveHandler {

    private final LinearOpMode driveOpMode;

    public IMU imu;
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private Servo piston = null;

    //IMU imu = driveOpMode.hardwareMap.get(IMU.class, "imu");

    public driveHandler(LinearOpMode opmode) {driveOpMode = opmode;}
    public void boot() {
        //front left and right, back left and right
        IMU imu = driveOpMode.hardwareMap.get(IMU.class, "imu");
        FL = driveOpMode.hardwareMap.get(DcMotor.class, "FL");
        FR = driveOpMode.hardwareMap.get(DcMotor.class, "FR");
        BL = driveOpMode.hardwareMap.get(DcMotor.class, "BL");
        BR = driveOpMode.hardwareMap.get(DcMotor.class, "BR");
        piston = driveOpMode.hardwareMap.get(Servo.class, "piston");

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


        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);


    }
    public void botDrive(double y,double x,double rx, double speed){
        //this drive code is FIELD CENTRIC, if you want to switch it to ROBOT CENTRIC, replace the rot variable with their regular versions
        //get bot heading
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //adjust values to account for bot heading
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        //set values to account for speed
        rotX = rotX * speed;
        rotY = rotY * speed;
        rx = rx * speed;

        //calculate motor powers
        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double FLpower = (rotY + rotX + rx) / denom;
        double FRpower = (rotY - rotX - rx) / denom;
        double BLpower = (rotY - rotX + rx) / denom;
        double BRpower = (rotY + rotX - rx) / denom;

        //set motor powers
        FL.setPower(FLpower);
        FR.setPower(FRpower);
        BL.setPower(BLpower);
        BR.setPower(BRpower);
    }

    public void botControl(int modeNum){
        //handle actions outside of driving
        if(modeNum == 0){

        }
    }
}
