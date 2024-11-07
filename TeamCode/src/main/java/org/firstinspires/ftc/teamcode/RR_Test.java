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

import static org.firstinspires.ftc.teamcode.RR_Test_Constants.maxAccel;
import static org.firstinspires.ftc.teamcode.RR_Test_Constants.minAccel;
import static org.firstinspires.ftc.teamcode.RR_Test_Constants.moveSpeed;
import static org.firstinspires.ftc.teamcode.RR_Test_Constants.turnAngle;
import static org.firstinspires.ftc.teamcode.RR_Test_Constants.waitTime;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="RR Test", group="Linear OpMode")
//@Disabled
public class RR_Test extends LinearOpMode {
    Pose2d initialPose = new Pose2d(0, 0, 0);
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        VelConstraint velConstraint = new VelConstraint() {
            @Override
            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return moveSpeed;
            }
        };

        AccelConstraint accelConstraint = new AccelConstraint() {
            @NonNull
            @Override
            public MinMax minMaxProfileAccel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return new MinMax(minAccel, maxAccel);
            }
        };

        TurnConstraints turnConstraints = new TurnConstraints(RR_Test_Constants.turnSpeed, RR_Test_Constants.minTurnAccel, RR_Test_Constants.maxTurnAccel);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (!RR_Test_Constants.turnTest) {
                Actions.runBlocking(
                        drive.actionBuilder(initialPose)
    //                    drive.actionBuilder(drive.pose) //Restart from our current pose
                                .strafeToConstantHeading(new Vector2d(48, 0), velConstraint, accelConstraint)
    //                            .strafeTo(new Vector2d(48, 0), velConstraint, accelConstraint)
                                .waitSeconds(waitTime)
                                .strafeToConstantHeading(new Vector2d(24, 24), velConstraint, accelConstraint)
    //                            .strafeTo(new Vector2d(24, 24), velConstraint, accelConstraint)
                                .waitSeconds(waitTime)
                                .strafeToConstantHeading(new Vector2d(0, 0), velConstraint, accelConstraint)
    //                            .strafeTo(new Vector2d(0, 0), velConstraint, accelConstraint)
                                .waitSeconds(waitTime)
                                .turnTo(0, turnConstraints)
                                .waitSeconds(waitTime)
    //                            .turn(turnAngle)
                                .build());
            } else {
                Actions.runBlocking(
                        drive.actionBuilder(initialPose)
                                .turn(turnAngle, turnConstraints)
                                .waitSeconds(waitTime)
//                                .turnTo(Math.PI / 2, turnConstraints)
//                                .waitSeconds(waitTime)
//                                .turnTo(Math.PI, turnConstraints)
//                                .waitSeconds(waitTime)
//                                .turnTo(3 * Math.PI / 2, turnConstraints)
//                                .waitSeconds(waitTime)
//                                .turnTo(0, turnConstraints)
//                                .waitSeconds(waitTime)
                                .build());

                telemetry.addData("Heading", drive.pose.heading.toDouble());
                telemetry.update();
            }
        }
    }
}
