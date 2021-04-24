package org.valkyrienskies.kronch

import org.joml.Quaterniond
import org.joml.Vector3d
import org.joml.Vector3i
import org.joml.Vector3ic
import org.valkyrienskies.kronch.JointType.SPHERICAL
import org.valkyrienskies.kronch.collision.shapes.PlaneShape
import org.valkyrienskies.kronch.collision.shapes.SphereShape
import org.valkyrienskies.kronch.collision.shapes.VoxelShape

class PhysicsWorld {
    val bodies: MutableList<Body> = ArrayList()
    private val joints: MutableList<Joint> = ArrayList()

    init {
        // region Create bodies
        val boxSize = Vector3d(1.0, 1.0, 1.0)

        val groundBodyVoxels = ArrayList<Vector3ic>()
        for (x in -10..10) {
            for (z in -10..10) {
                groundBodyVoxels.add(Vector3i(x, 0, z))
            }
        }

        for (x in -2..2) {
            for (z in -2..2) {
                groundBodyVoxels.add(Vector3i(x, 1, z))
            }
        }

        // groundBodyVoxels.add(Vector3i(0, 2, 0))

        val singleVoxelShape = VoxelShape(listOf(Vector3i()))

        val biggerShapeVoxels = ArrayList<Vector3ic>()

        for (x in -1..1) {
            for (z in -1..1) {
                biggerShapeVoxels.add(Vector3i(x, 0, z))
            }
        }
        biggerShapeVoxels.add(Vector3i(0, -1, 0))

        val biggerVoxelShape = VoxelShape(biggerShapeVoxels)

        val sphereShape = SphereShape(.5)

        val firstBoxPose = Pose(Vector3d(0.0, 4.0, 0.0), Quaterniond())
        val firstBoxBody = Body(firstBoxPose)
        firstBoxBody.setBox(boxSize)
        firstBoxBody.shape = sphereShape

        val secondBoxPose = Pose(Vector3d(0.0, 6.0, 0.0), Quaterniond())
        val secondBoxBody = Body(secondBoxPose)
        secondBoxBody.setBox(boxSize)
        secondBoxBody.shape = sphereShape

        val thirdBoxPose = Pose(Vector3d(0.0, 8.0, 0.0), Quaterniond())
        val thirdBoxBody = Body(thirdBoxPose)
        thirdBoxBody.setBox(boxSize)
        thirdBoxBody.shape = sphereShape

        val groundPose = Pose(Vector3d(0.0, 0.0, 0.0), Quaterniond().rotateAxis(Math.toRadians(0.0), 0.0, 1.0, 1.0))
        val groundBody = Body(groundPose)
        groundBody.setBox(boxSize)
        groundBody.shape = PlaneShape(Vector3d(0.0, 1.0, 0.0))// VoxelShape(groundBodyVoxels)
        groundBody.isStatic = true

        // endregion

        // region Create joins
        val firstBoxToCeilingJoint =
            Joint(
                SPHERICAL, null, firstBoxBody, Pose(Vector3d(0.0, 4.5, 0.0), Quaterniond()),
                Pose(Vector3d(0.5, .5, 0.5), Quaterniond())
            )

        val firstBoxToSecondBoxJoint =
            Joint(
                SPHERICAL, firstBoxBody, secondBoxBody, Pose(Vector3d(-0.5, -.5, -0.5), Quaterniond()),
                Pose(Vector3d(0.5, -1.5, 0.5), Quaterniond())
            )

        val secondBoxToThirdBoxJoint =
            Joint(
                SPHERICAL, secondBoxBody, thirdBoxBody, Pose(Vector3d(.5, .5, .5), Quaterniond()),
                Pose(Vector3d(0.5, -1.5, 0.5), Quaterniond())
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
        // bodies.add(firstBoxBody)
        // bodies.add(secondBoxBody)
        // bodies.add(thirdBoxBody)

        var count = 0
        for (x in -3..3) {
            for (y in 3..6) {
                for (z in -3..3) {
                    val idk = Pose(
                        Vector3d(x.toDouble() + .01 * count, y.toDouble(), z.toDouble()), Quaterniond()
                    )
                    val idk2 = Body(idk)
                    idk2.setBox(boxSize)
                    idk2.shape = SphereShape(.5 * Math.random())
                    bodies.add(idk2)
                    count++
                }
            }
        }

        run {
            val idk0 = Pose(Vector3d(1.0, 0.0, 0.0), Quaterniond().rotateAxis(Math.toRadians(40.0), 0.0, 0.0, 1.0))
            val idkBody = Body(idk0)
            idkBody.setBox(boxSize)
            idkBody.shape = PlaneShape(Vector3d(0.0, 1.0, 0.0))// VoxelShape(groundBodyVoxels)
            idkBody.isStatic = true
            bodies.add(idkBody)
        }

        run {
            val idk0 = Pose(Vector3d(-1.0, 0.0, 0.0), Quaterniond().rotateAxis(Math.toRadians(-40.0), 0.0, 0.0, 1.0))
            val idkBody = Body(idk0)
            idkBody.setBox(boxSize)
            idkBody.shape = PlaneShape(Vector3d(0.0, 1.0, 0.0))// VoxelShape(groundBodyVoxels)
            idkBody.isStatic = true
            bodies.add(idkBody)
        }

        // joints.add(firstBoxToCeilingJoint)
        // joints.add(firstBoxToSecondBoxJoint)
        // joints.add(secondBoxToThirdBoxJoint)
    }

    fun simulate(timeStep: Double) {
        val gravity = Vector3d(0.0, -10.0, 0.0)
        val numSubsteps = 40
        simulate(bodies, joints, timeStep, numSubsteps, gravity)

        // val groundBody = bodies[0]
        // groundBody.pose.q.rotateY(timeStep * Math.PI / 4.0)
        // groundBody.pose.q.normalize()
        // groundBody.quaternion.set(groundBody.pose.q)
        // groundBody.omega.set(0.0, Math.PI / 4.0, 0.0)
    }
}
