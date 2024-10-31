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

    // 裏（ボタンAが北）
    custom.estimate(
        [input.acceleration(Dimension.X), input.acceleration(Dimension.Y), input.acceleration(Dimension.Z)],
        [input.magneticForce(Dimension.X), input.magneticForce(Dimension.Y), input.magneticForce(Dimension.Z)]
    )

    // // 平（ロゴが北)
    // custom.estimate(
    //     [input.acceleration(Dimension.Y), input.acceleration(Dimension.X), -input.acceleration(Dimension.Z)],
    //     [input.magneticForce(Dimension.Y), input.magneticForce(Dimension.X), -input.magneticForce(Dimension.Z)]
    // )

    // // 縦（ウラが北)
    // custom.estimate(
    //     [input.acceleration(Dimension.Z), input.acceleration(Dimension.X), input.acceleration(Dimension.Y)],
    //     [input.magneticForce(Dimension.Z), input.magneticForce(Dimension.X), input.magneticForce(Dimension.Y)]
    // )

    serial.writeNumbers(custom.quaternion())
    
    // serial.writeValue("azi", custom.getAzimuthRadians())
    // serial.writeValue("yaw", custom.getYawRadians())
    // serial.writeValue("pit", custom.getPitchRadians())
    // serial.writeValue("rol", custom.getRollRadians())
    
    serial.writeValue("azi", custom.getAzimuth())
    serial.writeValue("yaw", custom.getYaw())
    serial.writeValue("pit", custom.getPitch())
    serial.writeValue("rol", custom.getRoll())

    basic.pause(100)
})
