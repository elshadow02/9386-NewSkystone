package org.firstinspires.ftc.teamcode.PIDControl

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

/*
 * Created by Ethan L. 1-28-2020
 *
 * OpMode for tuning the PID for an arm that uses a single motor. Utilizes the Dashboard.
 *
 * Start the program in the Dashboard and edit the variables. Once you are ready to begin, change
 * the start variable to equal 1 and graph the current position against the target (angle). Adjust
 * the PID variables until the current position reaches the target without oscillation.
 */

@Disabled
@TeleOp(name = "ArmPIDKotlin")
@Config
class ArmPIDTunerKotlin : OpMode() {

    var arm: DcMotor? = null

    var pid = PIDController(kp, ki, kd)

    //Motor encoder ticks per angle of rotation
    var ticksPerAngle = 1.0

    internal var startMotorPos: Double = 0.toDouble()

    var loopCount = 0


    override// @Override tells the computer we intend to override OpMode's method init()
    fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        arm = hardwareMap.get(DcMotor::class.java, "arm")
        arm!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        arm!!.mode = DcMotor.RunMode.RUN_USING_ENCODER

        ticksPerAngle = arm!!.motorType.ticksPerRev * gearRatio / 360

        startMotorPos = arm!!.currentPosition.toDouble()
    }

    override fun loop() {
        pid.setkP(kp)
        pid.setkI(ki)
        pid.setkD(kd)

        telemetry.addData("Distance to travel: ", travel)
        telemetry.addData("Encoder Value: ", arm!!.currentPosition)
        telemetry.update()

        if (start == 1) {
            setPosition(travel.toDouble())
        }
    }

    fun setPosition(angle: Double) {
        var angle = angle
        angle *= ticksPerAngle

        val output: Double

        val error: Double


        error = angle - (arm!!.currentPosition - startMotorPos)

        //output = error * kp;
        output = pid.calculate(error)

        arm!!.power = output

        telemetry.addData("Encoder target: ", angle)
        telemetry.addData("Error: ", error)
        telemetry.addData("Output: ", output)
        telemetry.addData("kp: ", kp)
        telemetry.addData("ki: ", ki)
        telemetry.addData("kd: ", kd)
        telemetry.addData("loop count: ", loopCount)
        telemetry.update()
        loopCount += 1
    }

    companion object {

        var travel = 90

        var maxPower = 1.0

        var kp = 1.0
        var ki = 0.0
        var kd = 0.0

        //If you are using a gear system between the arm motor and the output shaft,
        //Put the overall gear ratio here.
        var gearRatio = 1.0

        var start = 0
    }

}

