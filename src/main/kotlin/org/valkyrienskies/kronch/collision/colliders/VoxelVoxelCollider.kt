package org.valkyrienskies.kronch.collision.colliders

import org.joml.Matrix4d
import org.joml.Quaterniond
import org.joml.Vector3d
import org.joml.Vector3dc
import org.joml.primitives.AABBd
import org.valkyrienskies.kronch.Pose
import org.valkyrienskies.kronch.collision.CollisionPair
import org.valkyrienskies.kronch.collision.CollisionPairc
import org.valkyrienskies.kronch.collision.CollisionResult
import org.valkyrienskies.kronch.collision.CollisionResultc
import org.valkyrienskies.kronch.collision.shapes.VoxelShape
import org.valkyrienskies.kronch.collision.shapes.VoxelShape.CollisionVoxelType.AIR
import org.valkyrienskies.kronch.collision.shapes.VoxelShape.CollisionVoxelType.PROXIMITY
import org.valkyrienskies.kronch.collision.shapes.getProjectionAlongAxis
import kotlin.math.roundToInt

object VoxelVoxelCollider : Collider<VoxelShape, VoxelShape> {
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
            forEachCorner { xCorner: Int, yCorner: Int, zCorner: Int ->
                run {
                    val pointPosInBody1Coordinates: Vector3dc = Vector3d(
                        surfaceVoxel.x() + xCorner * .25,
                        surfaceVoxel.y() + yCorner * .25,
                        surfaceVoxel.z() + zCorner * .25
                    )
                    val pointPosInBody0Coordinates: Vector3dc =
                        body1To0Transform.transformPosition(pointPosInBody1Coordinates, Vector3d())

                    val pointSize = .25

                    val pointAsAABB = AABBd(
                        pointPosInBody0Coordinates.x() - pointSize,
                        pointPosInBody0Coordinates.y() - pointSize,
                        pointPosInBody0Coordinates.z() - pointSize,
                        pointPosInBody0Coordinates.x() + pointSize,
                        pointPosInBody0Coordinates.y() + pointSize,
                        pointPosInBody0Coordinates.z() + pointSize
                    )

                    for (gridX in pointAsAABB.minX.roundToInt()..pointAsAABB.maxX.roundToInt()) {
                        for (gridY in pointAsAABB.minY.roundToInt()..pointAsAABB.maxY.roundToInt()) {
                            for (gridZ in pointAsAABB.minZ.roundToInt()..pointAsAABB.maxZ.roundToInt()) {
                                val voxelType = body0Shape.getCollisionVoxelType(gridX, gridY, gridZ)
                                if (voxelType == AIR || voxelType == PROXIMITY) continue // skip

                                val minNormal = Vector3d()
                                var minDepth = Double.MAX_VALUE

                                val body1Voxel =
                                    AABBd(gridX - .5, gridY - .5, gridZ - .5, gridX + .5, gridY + .5, gridZ + .5)

                                body0Shape.forEachAllowedNormal(
                                    gridX, gridY, gridZ
                                ) { normalX: Double, normalY: Double, normalZ: Double ->
                                    run forEachNormal@{
                                        val point0AsBBRange = getProjectionAlongAxis(
                                            pointAsAABB, normalX, normalY, normalZ, CollisionRange.create()
                                        )
                                        val body1VoxelAsRange = getProjectionAlongAxis(
                                            body1Voxel, normalX, normalY, normalZ, CollisionRange.create()
                                        )

                                        val overlap = body1VoxelAsRange.max - point0AsBBRange.min

                                        if (overlap < 0) {
                                            return@forEachNormal // No collision
                                        }

                                        if (overlap < minDepth) {
                                            minDepth = overlap
                                            minNormal.set(normalX, normalY, normalZ)
                                        }
                                    }
                                }

                                if (minDepth == Double.MAX_VALUE) continue // No collision

                                val body1CollisionPointInBody0Coordinates =
                                    Vector3d(pointPosInBody0Coordinates) // .fma(-.25, minNormal)
                                val body0CollisionPointInBody0Coordinates =
                                    Vector3d(body1CollisionPointInBody0Coordinates).fma(-minDepth, minNormal)

                                val normalInGlobalCoordinates = body0Transform.rotate(Vector3d(minNormal)).mul(-1.0)

                                val body1CollisionPointInBody1Coordinates = body1Transform.invTransform(
                                    body0Transform.transform(
                                        Vector3d(body1CollisionPointInBody0Coordinates)
                                    )
                                )

                                val collisionPair = CollisionPair(
                                    body0CollisionPointInBody0Coordinates, body1CollisionPointInBody1Coordinates,
                                    normalInGlobalCoordinates
                                )

                                collisionPairs.add(collisionPair)
                            }
                        }
                    }
                }
            }
        }

        return CollisionResult(collisionPairs)
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
