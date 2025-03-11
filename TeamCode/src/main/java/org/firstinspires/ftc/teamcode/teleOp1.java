package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.driveHandler;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp

public abstract class teleOp1 extends LinearOpMode {
    driveHandler robot = new driveHandler(this);

    public void runOpMode(Gamepad gamepad1, Gamepad gamepad2) throws InterruptedException {

        robot.boot();
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;


        while (opModeIsActive()) {
            robot.botDrive(y, x, rx);
            if (gamepad1.right_stick_button) {
                driveHandler.imu.resetYaw();
            }
        }

    }
}