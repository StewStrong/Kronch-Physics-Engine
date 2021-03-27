package org.valkyrienskies.kronch

import org.joml.Quaterniond
import org.joml.Vector3d
import org.joml.Vector3i
import org.valkyrienskies.kronch.JointType.SPHERICAL
import org.valkyrienskies.kronch.collision.shapes.VoxelShape

class PhysicsWorld {
    val bodies: MutableList<Body> = ArrayList()
    private val joints: MutableList<Joint> = ArrayList()

    init {
        // region Create bodies
        val boxSize = Vector3d(1.0, 1.0, 1.0)

        val singleVoxelShape = VoxelShape(listOf(Vector3i()))

        val firstBoxPose = Pose(Vector3d(0.0, 3.0, 2.0), Quaterniond())
        val firstBoxBody = Body(firstBoxPose)
        firstBoxBody.setBox(boxSize)
        firstBoxBody.shape = singleVoxelShape

        val secondBoxPose = Pose(Vector3d(.8, 1.0, 0.1), Quaterniond())
        val secondBoxBody = Body(secondBoxPose)
        secondBoxBody.setBox(boxSize)
        secondBoxBody.shape = singleVoxelShape

        val thirdBoxPose = Pose(Vector3d(0.1, 5.0, 0.2), Quaterniond())
        val thirdBoxBody = Body(thirdBoxPose)
        thirdBoxBody.setBox(boxSize)
        thirdBoxBody.shape = singleVoxelShape

        val groundBodyVoxels = ArrayList<Vector3i>()
        for (x in -10..10) {
            for (z in -10..10) {
                groundBodyVoxels.add(Vector3i(x, 0, z))
            }
        }
        val groundPose = Pose(Vector3d(0.0, -5.0, 0.0))
        val groundBody = Body(groundPose)
        groundBody.setBox(boxSize)
        groundBody.shape = VoxelShape(groundBodyVoxels)
        groundBody.isStatic = true

        // thirdBoxBody.vel.set(10.0, 0.0, 0.0)
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
        bodies.add(groundBody)
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
