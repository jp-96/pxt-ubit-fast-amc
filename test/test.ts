/**
 * tests go here; this will not be compiled when this package is used as an extension.
 */
function updateAlpha() {
    if (9 < alpha) {
        alpha = 9
    } else if (1 > alpha) {
        alpha = 1
    }
    accelmagic.setLowPassFilterAlpha(alpha / 10)
    basic.showNumber(alpha)
}
input.onButtonPressed(Button.A, function () {
    alpha += -1
    updateAlpha()
})
input.onButtonPressed(Button.AB, function () {
    input.calibrateCompass()
})
input.onButtonPressed(Button.B, function () {
    alpha += 1
    updateAlpha()
})
let euler: number[] = []
let quat: number[] = []
let alpha = 0
alpha = 4
updateAlpha()
serial.redirectToUSB()
serial.setBaudRate(BaudRate.BaudRate115200)
serial.writeLine("Hello, Accel Magic!")

basic.forever(function () {

    // // RAW（North: A-button）
    // accelmagic.updateAcceleration(input.acceleration(Dimension.X), input.acceleration(Dimension.Y), input.acceleration(Dimension.Z))
    // accelmagic.updateMagneticForce(input.magneticForce(Dimension.X), input.magneticForce(Dimension.Y), input.magneticForce(Dimension.Z))

    // // Horizontal （North: Logo mark)
    accelmagic.updateAcceleration(input.acceleration(Dimension.Y), input.acceleration(Dimension.X), -input.acceleration(Dimension.Z))
    accelmagic.updateMagneticForce(input.magneticForce(Dimension.Y), input.magneticForce(Dimension.X), -input.magneticForce(Dimension.Z))

    // // Upright（North: Back side)
    // accelmagic.updateAcceleration(input.acceleration(Dimension.Z), input.acceleration(Dimension.X), input.acceleration(Dimension.Y))
    // accelmagic.updateMagneticForce(input.magneticForce(Dimension.Z), input.magneticForce(Dimension.X), input.magneticForce(Dimension.Y))

})

basic.forever(function () {
    // estimate
    quat = accelmagic.estimateQuaternion()

    // logging
    serial.writeString("quaternion:")
    serial.writeNumbers(quat)
    euler = accelmagic.quaternionToEulerAngles(quat)

    // serial.writeValue("AZMT(rad)", accelmagic.getAzimuthRadians(euler))
    // serial.writeValue("HEAD(rad)", accelmagic.getHeadingRadians(euler))
    // serial.writeValue("PTCH(rad)", accelmagic.getPitchRadians(euler))
    // serial.writeValue("BANK(rad)", accelmagic.getBankRadians(euler))
    
    serial.writeValue("AZMT(deg)", accelmagic.getAzimuth(euler))
    serial.writeValue("HEAD(deg)", accelmagic.getHeading(euler))
    serial.writeValue("PITC(deg)", accelmagic.getPitch(euler))
    serial.writeValue("BANK(deg)", accelmagic.getBank(euler))
    
    basic.pause(200)
})
