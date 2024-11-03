namespace accelmagicfamc_ {

    // Accelerration (normalized)
    let ax = 0.0;
    let ay = 0.0;
    let az = 1.0;

    // Magnetic force (normalized)
    let mx = 1.0;
    let my = 0.0;
    let mz = 0.0;

    // Quaternion for simulator
    let q_ = [1.0, 0.0, 0.0, 0.0];

    let alpha_ = 1.0;

    //% shim=accelmagicfamc_::setLowPassFilterAlpha
    export function setLowPassFilterAlpha(alpha: number): void {
        alpha_ = alpha;
    }

    //% shim=accelmagicfamc_::updateAcceleration
    export function updateAcceleration(x: number, y: number, z: number): void {
        let norm = Math.sqrt(x * x + y * y + z * z)
        if (0 < norm) {
            norm = 1 / norm;
            ax = x * norm;
            ay = y * norm;
            az = z * norm;
        }
    }

    //% shim=accelmagicfamc_::updateMagneticForce
    export function updateMagneticForce(x: number, y: number, z: number): void {
        let norm = Math.sqrt(x * x + y * y + z * z)
        if (0 < norm) {
            norm = 1 / norm;
            mx = x * norm;
            my = y * norm;
            mz = z * norm;
        }
    }

    function simuEstimate(): void {
        // Accelerration Only for simulator
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

    //% shim=accelmagicfamc_::estimate
    export function estimate(): void {
        // for simulator
        simuEstimate();
    }

    //% shim=accelmagicfamc_::getW
    export function getW(): number {
        return q_[0];
    }

    //% shim=accelmagicfamc_::getX
    export function getX(): number {
        return q_[1];
    }

    //% shim=accelmagicfamc_::getY
    export function getY(): number {
        return q_[2];
    }

    //% shim=accelmagicfamc_::getZ
    export function getZ(): number {
        return q_[3];
    }

}
