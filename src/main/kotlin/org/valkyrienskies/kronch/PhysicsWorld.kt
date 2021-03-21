package org.valkyrienskies.kronch

import org.joml.Quaterniond
import org.joml.Vector3d
import org.valkyrienskies.kronch.JointType.SPHERICAL

class PhysicsWorld {
    val bodies: MutableList<Body> = ArrayList()
    private val joints: MutableList<Joint> = ArrayList()

    init {
        // region Create bodies
        val boxSize = Vector3d(1.0, 1.0, 1.0)

        val firstBoxPose = Pose(Vector3d(0.0, 3.0, 0.0), Quaterniond())
        val firstBoxBody = Body(firstBoxPose)
        firstBoxBody.setBox(boxSize)

        val secondBoxPose = Pose(Vector3d(0.0, 2.0, 0.0), Quaterniond())
        val secondBoxBody = Body(secondBoxPose)
        firstBoxBody.setBox(boxSize)

        val thirdBoxPose = Pose(Vector3d(0.0, 1.0, 0.0), Quaterniond())
        val thirdBoxBody = Body(thirdBoxPose)
        thirdBoxBody.setBox(boxSize)

        thirdBoxBody.vel.set(10.0, 0.0, 0.0)
        // endregion

        // region Create joins
        val firstBoxToCeilingJoint =
            Joint(
                SPHERICAL, null, firstBoxBody, Pose(Vector3d(0.0, 3.0, 0.0), Quaterniond()),
                Pose(Vector3d(0.5, .5, 0.5), Quaterniond())
            )

        val firstBoxToSecondBoxJoint =
            Joint(
                SPHERICAL, firstBoxBody, secondBoxBody, Pose(Vector3d(0.5, -.5, 0.0), Quaterniond()),
                Pose(Vector3d(0.5, .5, 0.5), Quaterniond())
            )

        val secondBoxToThirdBoxJoint =
            Joint(
                SPHERICAL, secondBoxBody, thirdBoxBody, Pose(Vector3d(0.0, -.5, 0.0), Quaterniond()),
                Pose(Vector3d(0.5, .5, 0.5), Quaterniond())
            )

        // Add damping forces to the joints
        val jointRotDamping = 1.0
        val jointPosDamping = 1.0

        firstBoxToCeilingJoint.rotDamping = jointRotDamping
        firstBoxToCeilingJoint.posDamping = jointPosDamping

        firstBoxToSecondBoxJoint.rotDamping = jointRotDamping
        firstBoxToSecondBoxJoint.posDamping = jointPosDamping

        secondBoxToThirdBoxJoint.rotDamping = jointRotDamping
        secondBoxToThirdBoxJoint.posDamping = jointPosDamping

        // endregion

        bodies.add(firstBoxBody)
        bodies.add(secondBoxBody)
        bodies.add(thirdBoxBody)
        joints.add(firstBoxToCeilingJoint)
        joints.add(firstBoxToSecondBoxJoint)
        joints.add(secondBoxToThirdBoxJoint)
    }

    fun simulate(timeStep: Double) {
        val gravity = Vector3d(0.0, -10.0, 0.0)
        val numSubsteps = 40
        simulate(bodies, joints, timeStep, numSubsteps, gravity)
    }
}
