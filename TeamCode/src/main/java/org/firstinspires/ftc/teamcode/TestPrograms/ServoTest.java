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

package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Tester", group = "Tester")
@Disabled
public class ServoTest extends LinearOpMode {

    Servo servo;

    double startpos = 0.5;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "jewel_servo");
        stickPos prevpos = stickPos.Zero;
        double servopos = startpos;
        waitForStart();
        while(opModeIsActive()) {
//            if (gamepad1.left_stick_y > 0.2 && prevpos == stickPos.Zero) {
//                prevpos = stickPos.NotZero;
//                servopos++;
//                servo.setPosition(servopos);
//            } else if(gamepad1.left_stick_y < -0.2 && prevpos == stickPos.Zero) {
//                prevpos = stickPos.NotZero;
//                servopos--;
//                servo.setPosition(servopos);
//            } else {
//                prevpos = stickPos.Zero;
//            }
            if(gamepad1.y){
                servo.setPosition(.2);
            } else if(gamepad1.a){
                servo.setPosition(.6);
            } else if(gamepad1.b) {
                servo.setPosition(.3);
            } else if(gamepad1.x) {
                servo.setPosition(.5);
            }
            telemetry.addLine("Servo Position: " + servopos);
        }
    }
    public enum stickPos {
        NotZero, Zero;
    }

}
