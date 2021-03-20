package org.valkyrienskies.kronch

import org.joml.Vector3d

class PhysicsWorld {
    val bodies: MutableList<Body> = ArrayList()
    private val joints: MutableList<Joint> = ArrayList()

    init {
        val numObjects = 100
        val objectsSize = Vector3d(0.02, 0.04, 0.02)
        val lastObjectsSize = Vector3d(0.2, 0.04, 0.2)

        val rotDamping = 1000.0
        val posDamping = 1000.0

        val pos = Vector3d(0.0, (numObjects * objectsSize.y + lastObjectsSize.y) * 1.4 + 0.2, 0.0)
        val pose = Pose()
        var lastBody: Body? = null
        val jointPose0 = Pose()
        val jointPose1 = Pose()
        jointPose0.q.fromAxisAngleRad(0.0, 0.0, 1.0, 0.5 * Math.PI)
        jointPose1.q.fromAxisAngleRad(0.0, 0.0, 1.0, 0.5 * Math.PI)
        val lastSize = Vector3d(objectsSize)

        for (i in 0 until numObjects) {

            val size = if ((i < numObjects - 1)) objectsSize else lastObjectsSize

            // physics

            pose.p.set(pos.x, pos.y - i * objectsSize.y, pos.z)

            val boxBody = Body(pose)
            boxBody.setBox(size)
            bodies.add(boxBody)

            val s = if (i % 2 == 0) -0.5 else 0.5
            jointPose0.p.set(s * size.x, 0.5 * size.y, s * size.z)
            jointPose1.p.set(s * lastSize.x, -0.5 * lastSize.y, s * lastSize.z)

            if (lastBody != null) {
                jointPose1.copy(jointPose0)
                jointPose1.p.add(pose.p)
            }

            val joint = Joint(JointType.SPHERICAL, boxBody, lastBody, jointPose0, jointPose1)
            joint.rotDamping = rotDamping
            joint.posDamping = posDamping
            joints.add(joint)

            lastBody = boxBody
            lastSize.set(size)
        }
    }

    fun simulate(timeStep: Double) {
        val gravity = Vector3d(0.0, -10.0, 0.0)
        val numSubsteps = 40
        simulate(bodies, joints, timeStep, numSubsteps, gravity)
    }
}
