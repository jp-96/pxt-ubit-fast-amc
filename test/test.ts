/**
 * tests go here; this will not be compiled when this package is used as an extension.
 */
function updateAlpha() {
    if (9 < alpha) {
        alpha = 9
    } else if (1 > alpha) {
        alpha = 1
    }
    accelmagic.setAlpha(alpha / 100)
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
let rpy: accelmagic.EulerAngles = null
let quat: accelmagic.Quaternion = null
let alpha = 0
serial.redirectToUSB()
serial.setBaudRate(BaudRate.BaudRate115200)
basic.showString("AccelMagic!")
alpha = 4
updateAlpha()

basic.forever(function () {

    // RAW (North: A-button, upside-down)
    accelmagic.updateAcc(input.acceleration(Dimension.X), input.acceleration(Dimension.Y), input.acceleration(Dimension.Z))
    accelmagic.updateMag(input.magneticForce(Dimension.X), input.magneticForce(Dimension.Y), input.magneticForce(Dimension.Z))

    // // BASIC: a non-tilt compensated bearing of the device (North: logo mark)
    // accelmagic.updateAcceleration(input.acceleration(Dimension.Y), input.acceleration(Dimension.X), -input.acceleration(Dimension.Z))
    // accelmagic.updateMagneticForce(input.magneticForce(Dimension.Y), input.magneticForce(Dimension.X), -input.magneticForce(Dimension.Z))

    // // TILT: a tilt compensated bearing of the device (North: back side)
    // accelmagic.updateAcceleration(input.acceleration(Dimension.Z), input.acceleration(Dimension.X), input.acceleration(Dimension.Y))
    // accelmagic.updateMagneticForce(input.magneticForce(Dimension.Z), input.magneticForce(Dimension.X), input.magneticForce(Dimension.Y))

})

basic.forever(function () {

    // estimate
    quat = accelmagic.estimate()

    // logging - Quaternion
    serial.writeString("Q:")
    serial.writeNumbers(accelmagic.quatAsArray(quat))

    // RAW --> BASIC: calculates non-tilt compensated bearing of the device (North: logo mark)
    quat = accelmagic.multiply(quat, accelmagic.quat(0, 0.7, 0.7, 0))
    // RAW --> TILT: calculates tilt compensated bearing of the device (North: back side)
    //quat = accelmagic.multiplyQuats(quat, accelmagic.createQuat(-0.5, 0.5, 0.5, 0.5))

    // logging - EulerAngles
    rpy = accelmagic.quatToRpy(quat)

    serial.writeValue("A", accelmagic.intDeg(accelmagic.angle(rpy,AngleRPY.Azimuth)))
    serial.writeValue("Y", accelmagic.intDeg(accelmagic.angle(rpy,AngleRPY.Yaw)))
    serial.writeValue("P", accelmagic.intDeg(accelmagic.angle(rpy,AngleRPY.Pitch)))
    serial.writeValue("R", accelmagic.intDeg(accelmagic.angle(rpy,AngleRPY.Roll)))

    basic.pause(200)
})
