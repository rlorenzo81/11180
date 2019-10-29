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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Teleop Tank", group="Pushbot")
@Disabled
public class GyroTurnTest extends OpMode {

    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    private BNO055IMU imu;
    private double headingResetValue;

    public void init() {
        this.imu = (BNO055IMU)this.hardwareMap.get("imu"); //or whatever it is named in your configuration file
        final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        this.imu.initialize(parameters);
    }
    public void start() {
        this.headingResetValue = this.getAbsoluteHeading();
    }
    public void loop() {
        final double heading = this.getRelativeHeading();

        final boolean doneFORWARD =gyroCorrect(0,1,heading,0.1, 0.3)>10;
        final boolean doneTurn = this.gyroCorrect(90.0, 1.0, heading, 0.1, 0.3) > 10;


        //this.telemetry.addData("Heading", heading);
        //this.telemetry.addData("Done turn", doneTurn);
    }

    private double getAbsoluteHeading() {
        return this.imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    private double getRelativeHeading() {
        return this.getAbsoluteHeading() - this.headingResetValue;
    }

    //GYRO-CONTROLLED TURN CODE
    private int correctCount = 0;
    /**
     * @param gyroTarget The target heading in degrees, between 0 and 360
     * @param gyroRange The acceptable range off target in degrees, usually 1 or 2
     * @param gyroActual The current heading in degrees, between 0 and 360
     * @param minSpeed The minimum power to apply in order to turn (e.g. 0.05 when moving or 0.15 when stopped)
     * @param addSpeed The maximum additional speed to apply in order to turn (proportional component), e.g. 0.3
     * @return The number of times in a row the heading has been in the range
     */
    public int gyroCorrect(double gyroTarget, double gyroRange, double gyroActual, double minSpeed, double addSpeed) {
        double delta = (gyroTarget - gyroActual + 360.0) % 360.0; //the difference between target and actual mod 360
        if (delta > 180.0) delta -= 360.0; //makes delta between -180 and 180
        if (Math.abs(delta) > gyroRange) { //checks if delta is out of range
            this.correctCount = 0;
            double gyroMod = delta / 45.0; //scale from -1 to 1 if delta is less than 45 degrees
            if (Math.abs(gyroMod) > 1.0) gyroMod = Math.signum(gyroMod); //set gyromod to 1 or -1 if the error is more than 45 degrees
            this.turn(minSpeed * Math.signum(gyroMod) + addSpeed * gyroMod);
        }
        else {
            this.correctCount++;
            this.turn(0.0);
        }
        return this.correctCount;
    }
    public void turn(double sPower) {
        //put code here to turn the robot at the given power
        /*front_left.setPower(- sPower);
        back_left.setPower(- sPower);
        front_right.setPower(+ sPower);
        back_right.setPower(+ sPower);*/
       // robot.leftDrive.setPower(-sPower);
        //robot.rightDrive.setPower(+sPower);

    }
}