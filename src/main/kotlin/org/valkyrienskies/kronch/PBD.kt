package org.valkyrienskies.kronch

import org.joml.Quaterniond
import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.kronch.collision.CollisionResultc
import org.valkyrienskies.kronch.collision.colliders.VoxelVoxelCollider
import org.valkyrienskies.kronch.collision.shapes.BoxShape
import org.valkyrienskies.kronch.collision.shapes.CollisionShape
import org.valkyrienskies.kronch.collision.shapes.VoxelShape
import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.max

// pretty much one-for-one port of https://github.com/matthias-research/pages/blob/master/challenges/PBD.js

class Pose(
    val p: Vector3d = Vector3d(0.0, 0.0, 0.0),
    val q: Quaterniond = Quaterniond(0.0, 0.0, 0.0, 1.0)
) {

    fun copy(pose: Pose) {
        this.p.set(pose.p)
        this.q.set(pose.q)
    }

    fun clone() = Pose(Vector3d(p), Quaterniond(q))

    fun rotate(v: Vector3d): Vector3d {
        v.rotate(this.q)
        return v
    }

    fun invRotate(v: Vector3d) {
        this.q.transformInverse(v)
    }

    fun transform(v: Vector3d): Vector3d {
        v.rotate(this.q)
        v.add(this.p)
        return v
    }

    fun invTransform(v: Vector3d): Vector3d {
        v.sub(this.p)
        this.invRotate(v)
        return v
    }

    fun transformPose(pose: Pose) {
        pose.q.premul(this.q)
        this.rotate(pose.p)
        pose.p.add(this.p)
    }
}

fun getQuatAxis0(q: Quaterniond): Vector3d {
    val x2 = q.x * 2.0
    val w2 = q.w * 2.0
    return Vector3d((q.w * w2) - 1.0 + q.x * x2, (q.z * w2) + q.y * x2, (-q.y * w2) + q.z * x2)
}

fun getQuatAxis1(q: Quaterniond): Vector3d {
    val y2 = q.y * 2.0
    val w2 = q.w * 2.0
    return Vector3d((-q.z * w2) + q.x * y2, (q.w * w2) - 1.0 + q.y * y2, (q.x * w2) + q.z * y2)
}

fun getQuatAxis2(q: Quaterniond): Vector3d {
    val z2 = q.z * 2.0
    val w2 = q.w * 2.0
    return Vector3d((q.y * w2) + q.x * z2, (-q.x * w2) + q.y * z2, (q.w * w2) - 1.0 + q.z * z2)
}

const val maxRotationPerSubstep = 0.5

class Body(_pose: Pose) {

    val pose = _pose.clone()
    val prevPose = _pose.clone()
    val origPose = _pose.clone()

    val vel = Vector3d()
    val omega = Vector3d()

    // [prevVel] and [prevOmega] are used to compute friction
    val prevVel = Vector3d()
    val prevOmega = Vector3d()

    var invMass = 1.0
    val invInertia = Vector3d(1.0, 1.0, 1.0)

    val position = Vector3d(pose.p)
    val quaternion = Quaterniond(pose.q)

    var isStatic = false

    // Use a box shape by default
    var shape: CollisionShape = BoxShape(Vector3d(.5, .5, .5))

    fun setBox(size: Vector3d, density: Double = 1.0) {
        var mass = size.x * size.y * size.z * density
        invMass = 1.0 / mass
        mass /= 12.0

        invInertia.set(
            1.0 / (size.y * size.y + size.z * size.z) / mass,
            1.0 / (size.z * size.z + size.x * size.x) / mass,
            1.0 / (size.x * size.x + size.y * size.y) / mass
        )
    }

    fun applyRotation(rot: Vector3d, scale: Double = 1.0) {
        var scale = scale
        // safety clamping. This happens very rarely if the solver
        // wants to turn the body by more than 30 degrees in the
        // orders of milliseconds

        val maxPhi = 0.5
        val phi = rot.length()

        if (phi * scale > maxRotationPerSubstep)
            scale = maxRotationPerSubstep / phi

        val dq = Quaterniond(rot.x * scale, rot.y * scale, rot.z * scale, 0.0)
        dq.mul(this.pose.q)
        this.pose.q.set(
            this.pose.q.x + 0.5 * dq.x, this.pose.q.y + 0.5 * dq.y,
            this.pose.q.z + 0.5 * dq.z, this.pose.q.w + 0.5 * dq.w
        )
        this.pose.q.normalize()
    }

    fun integrate(dt: Double, gravity: Vector3d) {
        this.prevPose.copy(this.pose)
        this.vel.add(gravity.x * dt, gravity.y * dt, gravity.z * dt)
        this.pose.p.add(this.vel.x * dt, this.vel.y * dt, this.vel.z * dt)
        this.applyRotation(this.omega, dt)
    }

    fun update(dt: Double) {
        this.prevVel.set(this.vel)
        this.prevOmega.set(this.omega)

        this.pose.p.sub(this.prevPose.p, this.vel)

        this.vel.mul(1.0 / dt)

        val dq = this.pose.q.mul(this.prevPose.q.conjugate(), Quaterniond())

        this.omega.set(dq.x * 2.0 / dt, dq.y * 2.0 / dt, dq.z * 2.0 / dt)
        if (dq.w < 0.0)
            this.omega.set(-this.omega.x, -this.omega.y, -this.omega.z)

        this.omega.mul(1.0 - 1.0 * dt)
        this.vel.mul(1.0 - 1.0 * dt)

        this.position.set(this.pose.p)
        this.quaternion.set(this.pose.q)
    }

    fun getVelocityAt(pos: Vector3d): Vector3d {
        val vel = Vector3d(0.0, 0.0, 0.0)
        pos.sub(this.pose.p, vel)
        vel.cross(this.omega)
        this.vel.sub(vel, vel)
        return vel
    }

    fun getInverseMass(normal: Vector3dc, pos: Vector3dc? = null): Double {
        if (isStatic) throw IllegalStateException("Cannot get inverse mass of a static body")
        val n = Vector3d()
        if (pos == null)
            n.set(normal)
        else {
            pos.sub(this.pose.p, n)
            n.cross(normal)
        }
        this.pose.invRotate(n)
        var w = n.x * n.x * this.invInertia.x +
            n.y * n.y * this.invInertia.y +
            n.z * n.z * this.invInertia.z
        if (pos != null)
            w += this.invMass
        return w
    }

    fun applyCorrection(corr: Vector3dc, pos: Vector3dc? = null, velocityLevel: Boolean = false) {
        val dq = Vector3d()
        if (pos == null)
            dq.set(corr)
        else {
            if (velocityLevel)
                this.vel.add(corr.x() * this.invMass, corr.y() * this.invMass, corr.z() * this.invMass)
            else
                this.pose.p.add(corr.x() * this.invMass, corr.y() * this.invMass, corr.z() * this.invMass)
            pos.sub(this.pose.p, dq)
            dq.cross(corr)
        }
        this.pose.invRotate(dq)
        dq.set(
            this.invInertia.x * dq.x,
            this.invInertia.y * dq.y, this.invInertia.z * dq.z
        )
        this.pose.rotate(dq)
        if (velocityLevel)
            this.omega.add(dq)
        else
            this.applyRotation(dq)
    }
}

private const val PAIR_CORRECTION_MIN_LENGTH = 1e-10

fun applyBodyPairCorrection(
    body0: Body?, body1: Body?, corr: Vector3dc, compliance: Double,
    dt: Double, pos0: Vector3dc? = null, pos1: Vector3dc? = null, velocityLevel: Boolean = false
) {
    val C = corr.length()
    if (C == 0.0)
        return

    val normal = Vector3d(corr)
    normal.normalize()

    val w0 = if (body0 != null && !body0.isStatic) body0.getInverseMass(normal, pos0) else 0.0
    val w1 = if (body1 != null && !body1.isStatic) body1.getInverseMass(normal, pos1) else 0.0

    val w = w0 + w1
    if (w == 0.0)
        return

    val lambda = -C / (w + (compliance / dt / dt))
    normal.mul(-lambda)

    if (body0 != null && !body0.isStatic) {
        body0.applyCorrection(normal, pos0, velocityLevel)
    }

    if (body1 != null && !body1.isStatic) {
        normal.mul(-1.0)
        body1.applyCorrection(normal, pos1, velocityLevel)
    }
}

fun limitAngle(
    body0: Body?, body1: Body?, n: Vector3d, a: Vector3d, b: Vector3d, minAngle: Double, maxAngle: Double,
    compliance: Double, dt: Double, maxCorr: Double = Math.PI
) {
    // the key function to handle all angular joint limits
    val c = a.cross(b, Vector3d())

    var phi = asin(c.dot(n))
    if (a.dot(b) < 0.0)
        phi = Math.PI - phi

    if (phi > Math.PI)
        phi -= 2.0 * Math.PI
    if (phi < -Math.PI)
        phi += 2.0 * Math.PI

    if (phi < minAngle || phi > maxAngle) {
        phi = minAngle.coerceIn(phi, maxAngle)

        val q = Quaterniond()
        q.setAngleAxis(phi, n)

        val omega = Vector3d(a)
        omega.rotate(q)
        omega.cross(b)

        phi = omega.length()
        if (phi > maxCorr)
            omega.mul(maxCorr / phi)

        applyBodyPairCorrection(body0, body1, omega, compliance, dt)
    }
}

enum class JointType {
    SPHERICAL, HINGE, FIXED
}

class Joint(
    val type: JointType,
    val body0: Body?,
    val body1: Body?,
    _localPose0: Pose,
    _localPose1: Pose
) {

    val localPose0: Pose
    val localPose1: Pose
    val globalPose0: Pose
    val globalPose1: Pose

    val compliance = 0.0
    var rotDamping = 0.0
    var posDamping = 0.0
    val hasSwingLimits = false
    val minSwingAngle = -2.0 * Math.PI
    val maxSwingAngle = 2.0 * Math.PI
    val swingLimitsCompliance = 0.0
    val hasTwistLimits = false
    val minTwistAngle = -2.0 * Math.PI
    val maxTwistAngle = 2.0 * Math.PI
    val twistLimitCompliance = 0.0

    init {
        this.localPose0 = _localPose0.clone()
        this.localPose1 = _localPose1.clone()
        this.globalPose0 = _localPose0.clone()
        this.globalPose1 = _localPose1.clone()
    }

    fun updateGlobalPoses() {
        this.globalPose0.copy(this.localPose0)
        if (this.body0 != null)
            this.body0.pose.transformPose(this.globalPose0)
        this.globalPose1.copy(this.localPose1)
        if (this.body1 != null)
            this.body1.pose.transformPose(this.globalPose1)
    }

    fun solvePos(dt: Double) {

        this.updateGlobalPoses()

        // orientation

        if (this.type == JointType.FIXED) {
            val q = globalPose0.q
            q.conjugate()
            q.premul(globalPose1.q)
            val omega = Vector3d()
            omega.set(2.0 * q.x, 2.0 * q.y, 2.0 * q.z)
            // todo: wtf
            // if (omega.w < 0.0)
            //     omega.mul(-1.0)
            applyBodyPairCorrection(body0, body1, omega, this.compliance, dt)
        }

        if (this.type == JointType.HINGE) {

            // align axes
            val a0 = getQuatAxis0(this.globalPose0.q)
            val b0 = getQuatAxis1(this.globalPose0.q)
            val c0 = getQuatAxis2(this.globalPose0.q)
            val a1 = getQuatAxis0(this.globalPose1.q)
            a0.cross(a1)
            applyBodyPairCorrection(this.body0, this.body1, a0, 0.0, dt)

            // limits
            if (this.hasSwingLimits) {
                this.updateGlobalPoses()
                val n = getQuatAxis0(this.globalPose0.q)
                val b0 = getQuatAxis1(this.globalPose0.q)
                val b1 = getQuatAxis1(this.globalPose1.q)
                limitAngle(
                    this.body0, this.body1, n, b0, b1,
                    this.minSwingAngle, this.maxSwingAngle, this.swingLimitsCompliance, dt
                )
            }
        }

        if (this.type == JointType.SPHERICAL) {

            // swing limits
            if (this.hasSwingLimits) {
                this.updateGlobalPoses()
                val a0 = getQuatAxis0(this.globalPose0.q)
                val a1 = getQuatAxis0(this.globalPose1.q)
                val n = a0.cross(a1, Vector3d())
                n.normalize()
                limitAngle(
                    this.body0, this.body1, n, a0, a1,
                    this.minSwingAngle, this.maxSwingAngle, this.swingLimitsCompliance, dt
                )
            }
            // twist limits
            if (this.hasTwistLimits) {
                this.updateGlobalPoses()
                val n0 = getQuatAxis0(this.globalPose0.q)
                val n1 = getQuatAxis0(this.globalPose1.q)
                val n = n0.add(n1, Vector3d())
                n.normalize()

                val a0 = getQuatAxis1(this.globalPose0.q)
                val scale0 = -n.dot(a0)
                a0.add(n.x * scale0, n.y * scale0, n.z * scale0)
                a0.normalize()

                val a1 = getQuatAxis1(this.globalPose1.q)
                val scale1 = -n.dot(a1)

                a1.add(n.x * scale1, n.y * scale1, n.z * scale1)
                a1.normalize()

                // handling gimbal lock problem
                val maxCorr = if (n0.dot(n1) > -0.5) 2.0 * Math.PI else 1.0 * dt

                limitAngle(
                    this.body0, this.body1, n, a0, a1,
                    this.minTwistAngle, this.maxTwistAngle, this.twistLimitCompliance, dt, maxCorr
                )
            }
        }

        // position

        // simple attachment

        this.updateGlobalPoses()
        val corr = this.globalPose1.p.sub(this.globalPose0.p, Vector3d())
        applyBodyPairCorrection(
            this.body0, this.body1, corr, this.compliance, dt,
            this.globalPose0.p, this.globalPose1.p
        )
    }

    fun solveVel(dt: Double) {

        // Gauss-Seidel vals us make damping unconditionally stable in a
        // very simple way. We clamp the correction for each constraint
        // to the magnitude of the currect velocity making sure that
        // we never subtract more than there actually is.

        if (this.rotDamping > 0.0) {
            val omega = Vector3d(0.0, 0.0, 0.0)
            if (this.body0 != null)
                omega.sub(this.body0.omega)
            if (this.body1 != null)
                omega.add(this.body1.omega)
            omega.mul(Math.min(1.00, this.rotDamping * dt))
            applyBodyPairCorrection(
                this.body0, this.body1, omega, 0.0, dt,
                null, null, true
            )
        }
        if (this.posDamping > 0.0) {
            this.updateGlobalPoses()
            val vel = Vector3d(0.0, 0.0, 0.0)
            if (this.body0 != null)
                vel.sub(this.body0.getVelocityAt(this.globalPose0.p))
            if (this.body1 != null)
                vel.add(this.body1.getVelocityAt(this.globalPose1.p))
            vel.mul(Math.min(1.0, this.posDamping * dt))
            applyBodyPairCorrection(
                this.body0, this.body1, vel, 0.0, dt,
                this.globalPose0.p, this.globalPose1.p, true
            )
        }
    }
}

fun simulate(bodies: List<Body>, joints: List<Joint>, timeStep: Double, numSubsteps: Int, gravity: Vector3d) {
    val dt = timeStep / numSubsteps

    for (step in 0 until numSubsteps) {
        // Step 1, integrate velocity into position
        for (body in bodies)
            if (!body.isStatic) body.integrate(dt, gravity)

        // Step 2, solve positional constraints (like joints and contacts)
        for (joint in joints)
            joint.solvePos(dt)

        // Collide shapes with each other
        val collisions = solveCollisions(bodies)
        resolveCollisions(collisions, dt)

        // Step 3, compute new velocities given the positional updates
        for (body in bodies)
            if (!body.isStatic) body.update(dt)

        // Step 3.5, update velocities to apply friction and collision restitution
        correctRestitution(collisions, dt)

        // Step 4, solve velocity constraints
        for (joint in joints)
            joint.solveVel(dt)
    }
}

private fun correctRestitution(collisions: List<CollisionData>, dt: Double) {
    collisions.forEach { collision ->
        with(collision) {
            collisionResult.collisionPoints.forEach { collisionContact ->
                with(collisionContact) collisionContact@{
                    if (!used) {
                        // If this contact wasn't used, then it didn't effect velocity so we don't need to correct anything
                        return@collisionContact
                    }
                    // For each collision contact, set the relative velocity of the collision points on both bodies to 0
                    val body0CollisionPosInGlobal = body0.pose.transform(Vector3d(positionInFirstBody))
                    val body1CollisionPosInGlobal = body1.pose.transform(Vector3d(positionInSecondBody))

                    // Compute the current velocity along normal
                    val body0VelocityAtPoint = body0.getVelocityAt(body0CollisionPosInGlobal)
                    val body1VelocityAtPoint = body1.getVelocityAt(body1CollisionPosInGlobal)
                    val relativeVelocity = body0VelocityAtPoint.sub(body1VelocityAtPoint, Vector3d())
                    val relativeVelocityAlongNormal = normal.dot(relativeVelocity)

                    // Compute the previous velocity along normal
                    val relativeVelocityAlongNormalPrev = if (abs(relativeVelocityAlongNormal) > 2 * 10.0 * dt) {
                        val body0VelocityAtPointPrev = body0.getVelocityAt(body0CollisionPosInGlobal)
                        val body1VelocityAtPointPrev = body1.getVelocityAt(body1CollisionPosInGlobal)
                        val relativeVelocityPrev = body0VelocityAtPointPrev.sub(body1VelocityAtPointPrev, Vector3d())
                        normal.dot(relativeVelocityPrev)
                    } else {
                        0.0
                    }

                    // For now, just make collisions perfectly elastic
                    val coefficientOfRestitution = 1.0

                    // [deltaVelocity] serves 2 purposes:
                    // The first is to remove velocity added by [collision]
                    // The second is to apply the coefficient of restitution to [collision]
                    val deltaVelocity = normal.mul(
                        -relativeVelocityAlongNormal +
                            max(-coefficientOfRestitution * relativeVelocityAlongNormalPrev, 0.0),
                        Vector3d()
                    )

                    if (deltaVelocity.length() < PAIR_CORRECTION_MIN_LENGTH) {
                        // Small change, skip to avoid floating point error
                        return@collisionContact
                    }

                    applyBodyPairCorrection(
                        body0, body1, deltaVelocity, 1e-6, dt, body0CollisionPosInGlobal, body1CollisionPosInGlobal,
                        true
                    )

                    // Test code to measure relative velocity after the correction
                    /*
                    val body0VelocityAtPointFinal = body0.getVelocityAt(body0CollisionPosInGlobal)
                    val body1VelocityAtPointFinal = body1.getVelocityAt(body1CollisionPosInGlobal)

                    val relativeVelocityFinal = body0VelocityAtPointFinal.sub(body1VelocityAtPointFinal, Vector3d())

                    val relativeVelocityAlongNormalFinal = normal.dot(relativeVelocityFinal)

                    if (abs(relativeVelocityAlongNormalFinal) > 1e-10) {
                        println("error")
                    }
                     */
                }
            }
        }
    }
}

private fun resolveCollisions(collisions: List<CollisionData>, dt: Double, collisionCompliance: Double = 1e-4) {
    collisions.forEach { collision ->
        with(collision) {
            collisionResult.collisionPoints.forEach { collisionContact ->
                with(collisionContact) collisionContact@{
                    val body0PointPosInGlobal = body0.pose.transform(Vector3d(positionInFirstBody))
                    val body1PointPosInGlobal = body1.pose.transform(Vector3d(positionInSecondBody))

                    val positionDifference = body0PointPosInGlobal.sub(body1PointPosInGlobal, Vector3d())
                    val d = normal.dot(positionDifference)

                    if (d < PAIR_CORRECTION_MIN_LENGTH) {
                        // No longer colliding, skip this contact
                        // Technically any d value > 0 is valid, but don't allow values that are too small (< PAIR_CORRECTION_MIN_LENGTH) to avoid floating point errors
                        return@collisionContact
                    }

                    val corr = normal.mul(d, Vector3d())

                    // Compliance is set to [collisionCompliance] to dampen collision forces.
                    // Setting it to 0 makes it too strong, setting it too high makes collisions mushy.
                    // We use 1e-5 by default because its a good middle ground.
                    applyBodyPairCorrection(
                        body0, body1, corr, collisionCompliance, dt,
                        body0PointPosInGlobal, body1PointPosInGlobal, false
                    )
                    used = true
                }
            }
        }
    }
}

private data class CollisionData(val body0: Body, val body1: Body, val collisionResult: CollisionResultc)

private fun solveCollisions(bodies: List<Body>): List<CollisionData> {
    val collisionDataList = ArrayList<CollisionData>()
    for (i in bodies.indices) {
        for (j in i + 1 until bodies.size) {
            val body0 = bodies[i]
            val body1 = bodies[j]

            if (body0.isStatic and body1.isStatic) {
                continue // Both bodies are static, don't bother to collide with both of them
            }

            // For now assume both shapes are voxel shapes
            val collisionResult = VoxelVoxelCollider.computeCollisionBetweenShapes(
                body0.shape as VoxelShape, body0.pose, body1.shape as VoxelShape, body1.pose
            )

            if (collisionResult != null && collisionResult.colliding) {
                collisionDataList.add(CollisionData(body0, body1, collisionResult))
            }
        }
    }
    return collisionDataList
}
