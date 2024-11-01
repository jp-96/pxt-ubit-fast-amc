namespace Quaternion {
    ////////////////////////////////////////////////////////////////////////////
    //
    // 3D Math Primer for Games and Graphics Development
    //
    // Quaternion.cpp - Quaternion implementation
    //
    // Visit gamemath.com for the latest version of this file.
    //
    // For more details see section 11.3.
    //
    /////////////////////////////////////////////////////////////////////////////


    /////////////////////////////////////////////////////////////////////////////
    //
    // global data
    //
    /////////////////////////////////////////////////////////////////////////////

    // The global identity quaternion.  Notice that there are no constructors
    // to the Quaternion class, since we really don't need any.

    export const kQuaternionIdentity = [1.0, 0.0, 0.0, 0.0];

    //---------------------------------------------------------------------------
    // Quaternion::operator *
    //
    // Quaternion cross product, which concatonates multiple angular
    // displacements.  The order of multiplication, from left to right,
    // corresponds to the order that the angular displacements are
    // applied.  This is backwards from the *standard* definition of
    // quaternion multiplication.  See section 10.4.8 for the rationale
    // behind this deviation from the standard.

    export function multiply(q0: number[], q1: number[]): number[] {
        if ((4 != q0.length) || (4 != q1.length)) {
            return kQuaternionIdentity;
        }

        let w = q0[0];
        let x = q0[1];
        let y = q0[2];
        let z = q0[3];

        const aw = q1[0];
        const ax = q1[1];
        const ay = q1[2];
        const az = q1[3];

        w = w * aw - x * ax - y * ay - z * az;
        x = w * ax + x * aw + z * ay - y * az;
        y = w * ay + y * aw + x * az - z * ax;
        z = w * az + z * aw + y * ax - x * ay;

        return [w, x, y, z];
    }

    //---------------------------------------------------------------------------
    // Quaternion::normalize
    //
    // "Normalize" a quaternion.  Note that normally, quaternions
    // are always normalized (within limits of numerical precision).
    // See section 10.4.6 for more information.
    //
    // This function is provided primarily to combat floating point "error
    // creep," which can occur when many successive quaternion operations
    // are applied.

    export function normalize(q0: number[]): number[] {
        if (4 != q0.length) {
            return kQuaternionIdentity;
        }
        let w = q0[0];
        let x = q0[1];
        let y = q0[2];
        let z = q0[3];

        // Compute magnitude of the quaternion

        const mag = Math.sqrt(w * w + x * x + y * y + z * z);

        // Check for bogus length, to protect against divide by zero

        if (mag > 0.0) {

            // Normalize it

            const oneOverMag = 1.0 / mag;
            w *= oneOverMag;
            x *= oneOverMag;
            y *= oneOverMag;
            z *= oneOverMag;

            return [w, x, y, z];

        } else {

            // Houston, we have a problem
            // In a release build, just slam it to something

            return kQuaternionIdentity
        }
    }

    //---------------------------------------------------------------------------
    // conjugate
    //
    // Compute the quaternion conjugate.  This is the quaternian
    // with the opposite rotation as the original quaternian.  See 10.4.7

    export function conjugate(q0: number[]): number[] {
        if (4 != q0.length) {
            return kQuaternionIdentity;
        }

        // Same rotation amount

        const w = q0[0];

        // Opposite axis of rotation

        const x = -q0[1];
        const y = -q0[2];
        const z = -q0[3];

        // Return it

        return [w, x, y, z];
    }

}
