/**
 * tests go here; this will not be compiled when this package is used as an extension.
 */
/**
 * tests go here; this will not be compiled when this package is used as an extension.
 */
serial.redirectToUSB()
serial.setBaudRate(BaudRate.BaudRate115200)
serial.writeLine("Hello, Quaternion.")
serial.writeNumbers(custom.quaternion())
serial.writeLine("")
//custom.buildErr6(1,2,3,4,5,6)
basic.forever(function () {
    custom.setAcceleration(input.acceleration(Dimension.X), input.acceleration(Dimension.Y), input.acceleration(Dimension.Z))
    custom.setMagneticForce(input.magneticForce(Dimension.X), input.magneticForce(Dimension.Y), input.magneticForce(Dimension.Z))
    custom.estimate()
    serial.writeNumbers(custom.quaternion())
    serial.writeValue("yaw", custom.getYaw())
    serial.writeValue("pit", custom.getPitch())
    serial.writeValue("rol", custom.getRoll())
    basic.pause(100)
})
