package org.valkyrienskies.kronch.collision.colliders

import org.joml.Matrix4d
import org.joml.Quaterniond
import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.kronch.Pose
import org.valkyrienskies.kronch.collision.CollisionPair
import org.valkyrienskies.kronch.collision.CollisionPairc
import org.valkyrienskies.kronch.collision.CollisionResult
import org.valkyrienskies.kronch.collision.CollisionResultc
import org.valkyrienskies.kronch.collision.shapes.VoxelShape
import org.valkyrienskies.kronch.collision.shapes.VoxelShape.CollisionVoxelType.SURFACE
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt

object VoxelVoxelCollider : Collider<VoxelShape, VoxelShape> {

    private const val POINT_SPACING = .25
    private const val POINT_RADIUS = .25

    override fun computeCollisionBetweenShapes(
        body0Shape: VoxelShape, body0Transform: Pose, body1Shape: VoxelShape, body1Transform: Pose
    ): CollisionResultc {
        val collisionPairs = ArrayList<CollisionPairc>()

        val body1To0Transform = Matrix4d()

        body1To0Transform.rotate(
            Quaterniond(body0Transform.q).invert()
        )
        body1To0Transform.translate(
            body1Transform.p.x() - body0Transform.p.x(),
            body1Transform.p.y() - body0Transform.p.y(),
            body1Transform.p.z() - body0Transform.p.z()
        )
        body1To0Transform.rotate(body1Transform.q)

        for (surfaceVoxel in body1Shape.getSurfaceVoxels()) {
            forEachPointInCube(
                surfaceVoxel.x(), surfaceVoxel.y(), surfaceVoxel.z()
            ) forEachPointInBody1@{ pointX, pointY, pointZ ->
                val pointPosInBody1Coordinates: Vector3dc = Vector3d(pointX, pointY, pointZ)

                val pointPosInBody0Coordinates: Vector3dc = body0Transform.invTransform(
                    body1Transform.transform(
                        Vector3d(pointPosInBody1Coordinates)
                    )
                )

                val gridX = pointPosInBody0Coordinates.x().roundToInt()
                val gridY = pointPosInBody0Coordinates.y().roundToInt()
                val gridZ = pointPosInBody0Coordinates.z().roundToInt()


                for (testColX in gridX - 1..gridX + 1) {
                    for (testColY in gridY - 1..gridY + 1) {
                        for (testColZ in gridZ - 1..gridZ + 1) {
                            run forEachVoxel@{
                                val voxelType = body0Shape.getCollisionVoxelType(testColX, testColY, testColZ)

                                if (voxelType != SURFACE) return@forEachVoxel // No collision

                                val offsetX = pointPosInBody0Coordinates.x() - testColX
                                val offsetY = pointPosInBody0Coordinates.y() - testColY
                                val offsetZ = pointPosInBody0Coordinates.z() - testColZ

                                // The point on the box that is closest to the sphere
                                val boxClosestPointRelative = Vector3d(offsetX, offsetY, offsetZ)
                                val voxelSize: Vector3dc = Vector3d(.5, .5, .5)

                                boxClosestPointRelative.x = min(voxelSize.x(), boxClosestPointRelative.x())
                                boxClosestPointRelative.x = max(-voxelSize.x(), boxClosestPointRelative.x())
                                boxClosestPointRelative.y = min(voxelSize.y(), boxClosestPointRelative.y())
                                boxClosestPointRelative.y = max(-voxelSize.y(), boxClosestPointRelative.y())
                                boxClosestPointRelative.z = min(voxelSize.z(), boxClosestPointRelative.z())
                                boxClosestPointRelative.z = max(-voxelSize.z(), boxClosestPointRelative.z())

                                val boxClosestPointInBody0Coordinates: Vector3dc =
                                    Vector3d(boxClosestPointRelative).add(
                                        testColX.toDouble(), testColY.toDouble(), testColZ.toDouble()
                                    )

                                val collisionNormal = Vector3d(offsetX, offsetY, offsetZ).sub(boxClosestPointRelative)

                                val sphereRadius = .25

                                if (collisionNormal.lengthSquared() > sphereRadius * sphereRadius) {
                                    return@forEachVoxel // No collision
                                }

                                if (collisionNormal.length() < 1e-12) return@forEachVoxel // We can't normalize the normal!

                                val collisionDepth = sphereRadius - collisionNormal.length()

                                collisionNormal.normalize()

                                val body1CollisionPointInBody0Coordinates = Vector3d(pointPosInBody0Coordinates).sub(
                                    collisionNormal.x() * sphereRadius, collisionNormal.y() * sphereRadius,
                                    collisionNormal.z() * sphereRadius
                                )

                                val body1CollisionPointInBody1Coordinates = body1Transform.invTransform(
                                    body0Transform.transform(Vector3d(body1CollisionPointInBody0Coordinates))
                                )

                                val normalInGlobal = body0Transform.rotate(Vector3d(collisionNormal))

                                // if (!boxClosestPointInBody0Coordinates.isFinite || !body1CollisionPointInBody1Coordinates.isFinite || !normalInGlobal.isFinite) {
                                //     val bruh = 1
                                // }

                                collisionPairs.add(
                                    CollisionPair(
                                        boxClosestPointInBody0Coordinates, body1CollisionPointInBody1Coordinates,
                                        normalInGlobal.mul(1.0, Vector3d()),
                                        collisionDepth
                                    )
                                )
                            }
                        }
                    }
                }
            }
        }

        collisionPairs.sortBy { -it.originalDepth }

        if (collisionPairs.size > 3) {
            val i = 1
        }

        // Return up to 4 points
        while (collisionPairs.size > 4) collisionPairs.removeLast()

        return CollisionResult(collisionPairs)
    }

    private inline fun forEachPointInCube(
        voxelX: Int, voxelY: Int, voxelZ: Int, pointSpacing: Double = .25,
        function: (pointX: Double, pointY: Double, pointZ: Double) -> Unit
    ) {
        forEachCorner { xCorner, yCorner, zCorner ->
            function(
                voxelX.toDouble() + xCorner * pointSpacing,
                voxelY.toDouble() + yCorner * pointSpacing,
                voxelZ.toDouble() + zCorner * pointSpacing
            )
        }
    }

    private inline fun forEachCorner(function: (xCorner: Int, yCorner: Int, zCorner: Int) -> Unit) {
        for (i in 0 until 8) {
            val xCorner = if ((i shr 2) and 0x1 == 0x1) -1 else 1
            val yCorner = if ((i shr 1) and 0x1 == 0x1) -1 else 1
            val zCorner = if (i and 0x1 == 0x1) -1 else 1
            function(xCorner, yCorner, zCorner)
        }
        // Equivalent to the following, but much smaller bytecode
        /*
        function(1, 1, 1)
        function(1, 1, -1)
        function(1, -1, 1)
        function(1, -1, -1)
        function(-1, 1, 1)
        function(-1, 1, -1)
        function(-1, -1, 1)
        function(-1, -1, -1)
         */
    }
}
