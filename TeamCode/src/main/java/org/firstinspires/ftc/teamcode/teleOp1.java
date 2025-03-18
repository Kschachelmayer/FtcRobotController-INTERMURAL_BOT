package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.driveHandler;

@TeleOp

public class teleOp1 extends LinearOpMode {
    driveHandler robot = new driveHandler(this);


    public void runOpMode() throws InterruptedException {

        robot.boot();

        double driveSpeed = 0.5;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeIsActive()) {

            //Speed control
            if(gamepad1.cross){
                driveSpeed = 0.25;
            } else if(gamepad1.circle){
                driveSpeed = 0.5;
            } else if(gamepad1.square){
                driveSpeed = 0.75;
            } else if(gamepad1.triangle){
                driveSpeed = 1;
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            robot.botDrive(y, x, rx, driveSpeed);
            if (gamepad1.right_stick_button) {
                robot.imu.resetYaw();
            }

            //Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Speed", driveSpeed);
            telemetry.addLine();
            telemetry.addData("Heading", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

    }
}