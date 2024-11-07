/**
 * tests go here; this will not be compiled when this package is used as an extension.
 */
function updateAlpha() {
    if (9 < alpha) {
        alpha = 9
    } else if (1 > alpha) {
        alpha = 1
    }
    accelmagiq.setAlpha(alpha / 100)
    basic.showNumber(alpha)
}
input.onButtonPressed(Button.A, function () {
    alpha += -1
    updateAlpha()
})
input.onButtonPressed(Button.AB, function () {
    input.calibrateCompass()
    updateAlpha()
})
input.onButtonPressed(Button.B, function () {
    alpha += 1
    updateAlpha()
})
let rpy: accelmagiq.EulerAngles = null
let quat: accelmagiq.Quaternion = null
let alpha = 0
serial.redirectToUSB()
serial.setBaudRate(BaudRate.BaudRate115200)
basic.showString("AccelMagiQ!")
alpha = 4
updateAlpha()
accelmagiq.setEstimateMethod(accelmagiq.EstimationMethod.FAMC)
accelmagiq.setCoordinateSystem(accelmagiq.CoordinateSystem.BASIC)

basic.forever(function () {

    // RAW (North: A-button, upside-down)
    accelmagiq.updateAcc(input.acceleration(Dimension.X), input.acceleration(Dimension.Y), input.acceleration(Dimension.Z))
    accelmagiq.updateMag(input.magneticForce(Dimension.X), input.magneticForce(Dimension.Y), input.magneticForce(Dimension.Z))

    // // BASIC: a non-tilt compensated bearing of the device (North: logo mark)
    // accelmagiq.updateAcceleration(input.acceleration(Dimension.Y), input.acceleration(Dimension.X), -input.acceleration(Dimension.Z))
    // accelmagiq.updateMagneticForce(input.magneticForce(Dimension.Y), input.magneticForce(Dimension.X), -input.magneticForce(Dimension.Z))

    // // TILT: a tilt compensated bearing of the device (North: back side)
    // accelmagiq.updateAcceleration(input.acceleration(Dimension.Z), input.acceleration(Dimension.X), input.acceleration(Dimension.Y))
    // accelmagiq.updateMagneticForce(input.magneticForce(Dimension.Z), input.magneticForce(Dimension.X), input.magneticForce(Dimension.Y))

})

basic.forever(function () {

    // estimate
    quat = accelmagiq.quatFrom(accelmagiq.estimate())

    // logging - Quaternion
    serial.writeString("Q:")
    serial.writeNumbers(accelmagiq.quatAsArray(quat))

    // // RAW --> BASIC: calculates non-tilt compensated bearing of the device (North: logo mark)
    // quat = accelmagiq.multiply(quat, accelmagiq.quat(0, 0.7, 0.7, 0))
    // // RAW --> TILT: calculates tilt compensated bearing of the device (North: back side)
    // quat = accelmagiq.multiply(quat, accelmagiq.quat(-0.5, 0.5, 0.5, 0.5))

    // logging - EulerAngles
    rpy = accelmagiq.rpyFromQuat(quat)

    serial.writeValue("A", accelmagiq.intDeg(accelmagiq.angle(rpy, AngleRPY.Azimuth)))
    serial.writeValue("Y", accelmagiq.intDeg(accelmagiq.angle(rpy, AngleRPY.Yaw)))
    serial.writeValue("P", accelmagiq.intDeg(accelmagiq.angle(rpy, AngleRPY.Pitch)))
    serial.writeValue("R", accelmagiq.intDeg(accelmagiq.angle(rpy, AngleRPY.Roll)))

    basic.pause(200)
})
