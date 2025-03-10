package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.drive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp
public class teleOp1 extends LinearOpMode{
    drive robot = new drive(this);
    public void runOpMode(Gamepad gamepad1, Gamepad gamepad2) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if(gamepad1.options){
            imu.resetYaw();
        }
    }
}
