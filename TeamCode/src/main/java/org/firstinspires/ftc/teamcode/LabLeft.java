/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="LAB LEFT  ", group="DZTEAM")
//@Disabled
public class LabLeft extends LinearOpMode {

    /* Declare OpMode members. */
    MOSS robot           = new MOSS();   // Use a Pushbot's hardware
    private final double blueMin = 0.02;
    private final double orangeMin = 0.02;
    private double blueBasket ;
    private double orangeBasket ;
    private double potentiometreMinThreshhold = 1;
    private double getPotentiometreMaxThreshhold = 3;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        robot.swallower.setPower(robot.swallowerMaxPower);
        robot.polycord.setPower(robot.polycordMaxPower);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            left  = -gamepad1.left_stick_y + gamepad1.right_stick_x;
            right = -gamepad1.left_stick_y - gamepad1.right_stick_x;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            //moving the robot
            robot.forwardLeft.setPower(-left);
            robot.backwardLeft.setPower(-left);
            robot.forwardRight.setPower(right);
            robot.backwardRight.setPower(right);





            // Use gamepad buttons to move arm up (Y) and down (A)
            if (gamepad2.b)
                robot.rightBasket.setPosition(robot.servoOpenPosition);
            else if (gamepad2.x)
                robot.leftBasket.setPosition(-robot.servoOpenPosition);
            else{
                robot.rightBasket.setPosition(-robot.servoInitAngle);
                robot.leftBasket.setPosition(robot.servoInitAngle);
            }

            if (gamepad2.right_bumper) {
                robot.hangLeft.setPower(-robot.hangPositionMax);
                robot.hangRight.setPower(robot.hangPositionMax);
                robot.polycord.setPower(0);
            } else {
                if (gamepad2.left_bumper) {
                    robot.keephang = true;
                } else if (robot.keephang) {
                    robot.hangLeft.setPower(-robot.hangPositionMin);
                    robot.hangRight.setPower(robot.hangPositionMin);
                    robot.polycord.setPower(0);
                } else {
                    robot.hangLeft.setPower(0);
                    robot.hangRight.setPower(0);
                }
            }

            NormalizedRGBA color = robot.sensor.getNormalizedColors();
            telemetry.addData("blue ", color.blue);
            telemetry.addData("red ", color.red);
            telemetry.update();

            if (gamepad2.left_stick_button)

            { robot.sorter.setPosition(1);
                robot.polycord.setPower(0);
                telemetry.addData("blue ball detected ", color.blue);
            }

            else if ( gamepad2.right_stick_button){
                robot.sorter.setPosition(-1);
                robot.polycord.setPower(0);
                telemetry.addData("orange ball detected ", color.red);
            }
            else
            {
                robot.sorter.setPosition(robot.servoInitAngle);
                telemetry.addData("NO ball detected ", 0);
            }
        }






            if (gamepad1.left_bumper) {
                robot.polycord.setPower(0);
            } else {
                robot.polycord.setPower(1);
            }

            if (gamepad1.right_bumper) {
                robot.swallower.setPower(-1);
            } else {
                robot.swallower.setPower(1);
            }





            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(10);
            //idle();
        }

    }






