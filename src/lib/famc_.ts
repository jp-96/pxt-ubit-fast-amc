namespace famc_ {

    // Accelerration (normalized)
    let ax = 0.0;
    let ay = 0.0;
    let az = 1.0;

    // Magnetic force (normalized)
    let mx = 1.0;
    let my = 0.0;
    let mz = 0.0;

    // Quaternion for simulator
    let q_ = Quaternion.kQuaternionIdentity;

    //% shim=famc_::getW
    export function getW(): number {
        return q_[0];
    }

    //% shim=famc_::getX
    export function getX(): number {
        return q_[1];
    }

    //% shim=famc_::getY
    export function getY(): number {
        return q_[2];
    }

    //% shim=famc_::getZ
    export function getZ(): number {
        return q_[3];
    }

    let alpha_ = 1.0;

    //% shim=famc_::setLowPassFilterAlpha
    export function setLowPassFilterAlpha(alpha: number): void {
        alpha_ = alpha;
    }

    //% shim=famc_::updateAcceleration
    export function updateAcceleration(x: number, y: number, z: number): void {
        const norm = Math.sqrt(x * x + y * y + z * z)
        if (norm > 0) {
            ax = x / norm;
            ay = y / norm;
            az = z / norm;
        } else {
            ax = 0.0;
            ay = 0.0;
            az = 1.0;
        }
    }

    //% shim=famc_::updateMagneticForce
    export function updateMagneticForce(x: number, y: number, z: number): void {
        const norm = Math.sqrt(x * x + y * y + z * z)
        if (norm > 0) {
            mx = x / norm;
            my = y / norm;
            az = z / norm;
        } else {
            mx = 1.0;
            my = 0.0;
            mz = 0.0;
        }
    }

    function simuEstimate(): void {
        // Accelerration Only for simulator
        const qw = Math.sqrt((az + 1.0) / 2.0)
        const qx = ay / (2.0 * qw)
        const qy = -ax / (2.0 * qw)
        const qz = 0.0
        q_ = Quaternion.normalize([qw, qx, qy, qz]);
    }

    //% shim=famc_::estimate
    export function estimate(): void {
        // for simulator
        simuEstimate();
    }

}
