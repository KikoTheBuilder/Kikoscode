package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class MOSS
{
    /* Public OpMode members. */



    //motors
    public DcMotor  forwardLeft   = null;
    public DcMotor  forwardRight  = null;
    public DcMotor  backwardLeft    = null;
    public DcMotor  backwardRight = null;
    public DcMotor  swallower = null ;
    public DcMotor  polycord = null ;
    public DcMotor hangLeft = null ;
    public DcMotor hangRight = null;
    public AnalogInput petentiometre = null;

    //servos
    public Servo   leftBasket    = null;
    public Servo   rightBasket   = null;
    public Servo   sorter =     null ;

    //sensor
    public NormalizedColorSensor sensor =  null ;


    //constants
    public final double servoInitAngle = 0;
    public final double servoOpenPosition = 0;
    public final double hangPositionMax = 1.0;
    public final double hangPositionMin = -1;
    public final double swallowerMaxPower = 1;
    public final double polycordMaxPower = 1;
    public final double leftSorter = -1;
    public final double rightSorter = 1;
    public boolean keephang = false ;





    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public MOSS(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        forwardLeft   = hwMap.dcMotor.get("FoLe");
        forwardRight  = hwMap.dcMotor.get("FoRi");
        backwardLeft    = hwMap.dcMotor.get("BaLe");
        backwardRight = hwMap.dcMotor.get("BaRi");
        swallower = hwMap.dcMotor.get("THEMOTOR");
        hangLeft = hwMap.dcMotor.get("hangleft");
        hangRight = hwMap.dcMotor.get("hangright");
        polycord = hwMap.dcMotor.get("polycord");
        petentiometre = hwMap.analogInput.get("Petentiometre");


        //initialize sensor
        sensor = hwMap.get(NormalizedColorSensor.class , "colorsensor");


        //initialize servos
        sorter = hwMap.servo.get("sorter");
        leftBasket = hwMap.servo.get("leftbasket");
        rightBasket = hwMap.servo.get("rightbasket");

        // Set all motors to zero power
        forwardLeft.setPower(0);
        forwardRight.setPower(0);
        backwardLeft.setPower(0);
        backwardRight.setPower(0);
        swallower.setPower(0);
        polycord.setPower(0);
        hangLeft.setPower(0);
        hangRight.setPower(0);

        //set servos init angle
        sorter.setPosition(servoInitAngle);
        leftBasket.setPosition(servoInitAngle);
        rightBasket.setPosition(servoInitAngle);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.



        /*leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/




    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

