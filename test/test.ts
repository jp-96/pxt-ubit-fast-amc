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

    serial.writeString("quaternion:")
    serial.writeNumbers(custom.quaternion())
    
    serial.writeValue("AZMT(rad)", custom.getAzimuthRadians())
    serial.writeValue("HEAD(rad)", custom.getHeadingRadians())
    serial.writeValue("PTCH(rad)", custom.getPitchRadians())
    serial.writeValue("BANK(rad)", custom.getBankRadians())
    
    serial.writeValue("AZMT(deg)", custom.getAzimuth())
    serial.writeValue("HEAD(deg)", custom.getHeading())
    serial.writeValue("PITC(deg)", custom.getPitch())
    serial.writeValue("BANK(deg)", custom.getBank())

    basic.pause(100)
})
