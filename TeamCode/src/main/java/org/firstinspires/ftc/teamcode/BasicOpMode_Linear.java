/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

    //========================================
    // DECLARE OPMODE MEMBERS
    //========================================


    // Motors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfront = null;
    private DcMotor leftback = null;
    private DcMotor rightfront = null;
    private DcMotor rightback = null;
    private DcMotor clawmotor = null;
    private DcMotor stickmotor = null;

/*    // Servos and Start Position
    private Servo rightStick = null;
    private Servo leftStick = null;
    private static final double RIGHT_STICK_HOME = 0.0;
    private static final double LEFT_STICK_HOME = 0.0;

    //Min and Max the servo can move
    //RIGHTSTICK SERVO
    private static final double RIGHT_STICK_MIN_RANGE = 0.0;
    private static final double RIGHT_STICK_MAX_RANGE = 1.0;
    double rightStickposition = RIGHT_STICK_HOME;
    private static final double RIGHT_STICK_SPEED = 0.2;

    //LEFTSTICK SERVO
    private static final double LEFT_STICK_MIN_RANGE = 0.0;
    private static final double LEFT_STICK_MAX_RANGE = 1.0;
    double leftStickposition = RIGHT_STICK_HOME;
    private static final double LEFT_STICK_SPEED = 0.2;
**/

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //========================================
        // HARDWARE MAPPING
        //========================================

        // Initialize the hardware variables. Note that the strings used here as parameters
        // the name is the quotaton points are the names of the motors
        // step (using the FTC Robot Controller app on the phone).
        leftfront  = hardwareMap.get(DcMotor.class, "leftfront");
        leftback = hardwareMap.get(DcMotor.class, "leftback");
        rightfront  = hardwareMap.get(DcMotor.class, "rightfront");
        rightback = hardwareMap.get(DcMotor.class, "rightback");
        clawmotor = hardwareMap.get(DcMotor.class, "clawmotor");
        stickmotor = hardwareMap.get(DcMotor.class, "stickmotor);

  /*      //hardware mapping the SERVOS and giving them the position
        //RIGHTSTICK SERVO
        rightStick = hardwareMap.get(Servo.class,"rightStick");
        rightStick.setPosition(RIGHT_STICK_HOME);

        //LEFTSTICK SERVO
        leftStick = hardwareMap.get(Servo.class,"leftStick");
        leftStick.setPosition(LEFT_STICK_HOME);
**/

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);
        clawmotor.setDirection(DcMotor.Direction.FORWARD);
        stickmotor.setdirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //========================================
            // MECANUM DRIVETRAIN
            //========================================

            // Setup a variable for each drive wheel to save power level for telemetry
            double frontLeftPower;
            double backLeftPower;
            double frontRightPower;
            double backRightPower;
            double clawmotorPower;
            double stickmotorPower;

            // POV Mode uses left stick to go forward, and right stick to turn.
            // This uses basic math to combine motions and is easier to drive straight.
            double y = -gamepad1.left_stick_y;
            double x  =  gamepad1.right_stick_x;
            double turning = gamepad1.right_stick_x * 1.5;
           
            // here is the adapted calculated power on the motors
            // in addition to the two more added motors
            frontLeftPower    = Range.clip(y + x + turning, -1.0, 1.0) ;
            backLeftPower   = Range.clip(y - x + turning, -1.0, 1.0) ;
            frontRightPower   = Range.clip(y - x - turning, -1.0, 1.0) ;
            backRightPower   = Range.clip(y + x - turning, -1.0, 1.0) ;
            clawmotorPower   = Range.clip(y + x - turning, -1.0, 1.0) ; 
            stickmotorPower   = Range.clip(y + x - turning, -1.0, 1.0) ;
            // Send calculated power to wheels
            leftfront.setPower(frontLeftPower);
            leftback.setPower(backLeftPower);
            rightfront.setPower(frontRightPower);
            rightback.setPower(backRightPower);
            clawmotor.setPower(clawmotorPower);
            stickmotor.setPower(stickmotorPower);


            //========================================
            // GAMEPADS
            //========================================

            /*
             * This is the ideal place to put you code regarding the gamepads to control the robot
             * */

            // Servo Gamepad position and speed
            //RIGHTSTICK SERVO
         /*   if (gamepad1.y = true) {
                rightStickposition += RIGHT_STICK_SPEED;
            } else if (gamepad1.y = false) {
                rightStickposition -= RIGHT_STICK_SPEED;
            }

            //LEFTSTICK SERVO
            if (gamepad1.a = true) {
                leftStickposition += LEFT_STICK_SPEED;
            } else if (gamepad1.a = false) {
                leftStickposition -= LEFT_STICK_SPEED;
            }
            
            
            // Actual Movement of Servosright_stick_x
            //RIGHTSTICK SERVO
            rightStickposition = Range.clip(rightStickposition, RIGHT_STICK_MIN_RANGE, RIGHT_STICK_MAX_RANGE);
            rightStick.setPosition(rightStickposition);

            //LEFTSTICK SERVO
            leftStickposition = Range.clip(leftStickposition, LEFT_STICK_MIN_RANGE, LEFT_STICK_MAX_RANGE);
            leftStick.setPosition(leftStickposition);
**/
            
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("rightStick", "%.2f", rightStickposition);
            telemetry.addData("leftStick", "%.2f", leftStickposition);
            telemetry.update();


        }


    }

}
