namespace accelmagiq_ {

    // Accelerration (normalized) for simulator
    let ax = 0.0;
    let ay = 0.0;
    let az = 1.0;

    // Magnetic force (normalized) for simulator
    let mx = 1.0;
    let my = 0.0;
    let mz = 0.0;

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
            ax = x * norm;
            ay = y * norm;
            az = z * norm;
        }
    }

    //% shim=accelmagiq_::updateMagneticForce
    export function updateMagneticForce(x: number, y: number, z: number): void {
        // for simulator
        let norm = Math.sqrt(x * x + y * y + z * z)
        if (0 < norm) {
            norm = 1 / norm;
            mx = x * norm;
            my = y * norm;
            mz = z * norm;
        }
    }

    //% shim=accelmagiq_::estimate
    export function estimate(): void {
        // for simulator
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
