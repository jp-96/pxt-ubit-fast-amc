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
    let qw = 1.0;
    let qx = 0.0;
    let qy = 0.0;
    let qz = 0.0;

    //% shim=famc_::getW
    export function getW(): number {
        return qw;
    }

    //% shim=famc_::getX
    export function getX(): number {
        return qx;
    }

    //% shim=famc_::getY
    export function getY(): number {
        return qy;
    }

    //% shim=famc_::getZ
    export function getZ(): number {
        return qz;
    }

    //% shim=famc_::setAcceleration
    export function setAcceleration(x: number, y: number, z: number): void {
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

    //% shim=famc_::setMagneticForce
    export function setMagneticForce(x: number, y: number, z: number): void {
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
        qw = Math.sqrt((az + 1.0) / 2.0)
        qx = -ay / (2.0 * qw)
        qy = ax / (2.0 * qw)
        qz = 0.0
    }

    //% shim=famc_::estimate
    export function estimate(): void {
        // for simulator
        simuEstimate();
    }
    
}