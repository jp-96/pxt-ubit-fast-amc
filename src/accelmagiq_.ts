namespace accelmagiq_ {

    // Accelerration (normalized) for simulator
    let rawAx = 1.0;
    let rawAy = 0.0;
    let rawAz = 0.0;

    // Magnetic force (normalized) for simulator
    let rawMx = 1.0;
    let rawMy = 0.0;
    let rawMz = 0.0;

    // Quaternion for simulator
    let q_ = [1.0, 0.0, 0.0, 0.0];

    // coordinateSystem for simulator
    let coordinateSystem_ = 0;

    //% shim=accelmagiq_::setCoordinateSystem
    export function setCoordinateSystem(system: number) {
        // alpha for simulator
        coordinateSystem_ = system;
    }

    // alpha for simulator
    let alpha_ = 1.0;

    //% shim=accelmagiq_::setLowPassFilterAlpha
    export function setLowPassFilterAlpha(alpha: number): void {
        // for simulator
        alpha_ = alpha;
    }

    // method for simulator
    let method_ = 0;

    //% shim=accelmagiq_::setEstimateMethod
    export function setEstimateMethod(method: number): void {
        // for simulator
        method_ = method;
    }

    //% shim=accelmagiq_::updateAcceleration
    export function updateAcceleration(x: number, y: number, z: number): void {
        // for simulator
        let norm = Math.sqrt(x * x + y * y + z * z)
        if (0 < norm) {
            norm = 1 / norm;
            rawAx = x * norm;
            rawAy = y * norm;
            rawAz = z * norm;
        }
    }

    //% shim=accelmagiq_::updateMagneticForce
    export function updateMagneticForce(x: number, y: number, z: number): void {
        // for simulator
        let norm = Math.sqrt(x * x + y * y + z * z)
        if (0 < norm) {
            norm = 1 / norm;
            rawMx = x * norm;
            rawMy = y * norm;
            rawMz = z * norm;
        }
    }

    //% shim=accelmagiq_::estimate
    export function estimate(): void {
        // for simulator
        const ax = rawAy;
        const ay = rawAx;
        const az = -rawAz;

        let w = Math.sqrt((az + 1.0) / 2.0)
        let x = ay / (2.0 * w)
        let y = -ax / (2.0 * w)
        let z = 0.0
        let norm = Math.sqrt(w * w + x * x + y * y + z * z)
        if (0 < norm) {
            norm = 1 / norm;
            w *= norm;
            x *= norm;
            y *= norm;
            z *= norm;
            q_ = [w, x, y, z];
        }
    }

    //% shim=accelmagiq_::getW
    export function getW(): number {
        // for simulator
        return q_[0];
    }

    //% shim=accelmagiq_::getX
    export function getX(): number {
        // for simulator
        return q_[1];
    }

    //% shim=accelmagiq_::getY
    export function getY(): number {
        // for simulator
        return q_[2];
    }

    //% shim=accelmagiq_::getZ
    export function getZ(): number {
        // for simulator
        return q_[3];
    }

}
