package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class FirstTest extends OpMode {

    private boolean shouldStop = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Monty is skibidibidi bop");
    }
/*
    @Override
    public void loop() {
        if (gamepad1.b) {
            shouldStop = true;
        }

        if (shouldStop) {
            telemetry.addData("Status", "Stopping");
            telemetry.update();
            requestOpModeStop();
            return;
        }

        telemetry.addData("Status", "Running");
        telemetry.addData("Stop Condition", "Press gamepad1 B");
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
*/
@Override
public void loop() {
    double right_y=gamepad1.right_stick_y*-0.5;
    telemetry.addData("circle_button", gamepad1.circle);
    telemetry.addData("right_y", right_y);
    telemetry.addData("right_x", gamepad1.right_stick_x);
    telemetry.addData("left_x", gamepad1.left_stick_x);
    telemetry.addData("left_y", gamepad1.left_stick_y);
    telemetry.addData("x button", gamepad1.x);
    telemetry.update();
}

}