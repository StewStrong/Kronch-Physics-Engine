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
import org.valkyrienskies.kronch.collision.shapes.VoxelShape.CollisionVoxelType.AIR
import org.valkyrienskies.kronch.collision.shapes.VoxelShape.CollisionVoxelType.INTERIOR
import org.valkyrienskies.kronch.collision.shapes.VoxelShape.CollisionVoxelType.PROXIMITY
import org.valkyrienskies.kronch.collision.shapes.VoxelShape.CollisionVoxelType.SURFACE
import kotlin.math.abs
import kotlin.math.roundToInt

object VoxelVoxelCollider : Collider<VoxelShape, VoxelShape> {
    override fun computeCollisionBetweenShapes(
        body0Shape: VoxelShape, body0Transform: Pose, body1Shape: VoxelShape, body1Transform: Pose
    ): CollisionResultc? {
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

                    val gridX = pointPosInBody0Coordinates.x().roundToInt()
                    val gridY = pointPosInBody0Coordinates.y().roundToInt()
                    val gridZ = pointPosInBody0Coordinates.z().roundToInt()

                    val voxelType = body0Shape.getCollisionVoxelType(gridX, gridY, gridZ)

                    if (voxelType == AIR) {
                        // No collision
                        return@run
                    }

                    val minNormal = Vector3d()
                    val collisionPos = Vector3d()

                    body0Shape.forEachAllowedNormal(
                        gridX, gridY, gridZ
                    ) { normalX: Double, normalY: Double, normalZ: Double ->
                        run TestNormal@{
                            val normalProjection =
                                pointPosInBody0Coordinates.dot(
                                    normalX, normalY, normalZ
                                ) - (gridX * normalX + gridY * normalY + gridZ * normalZ)

                            val penetrationIntoVoxelSignedDistance = normalProjection + .25

                            val signedDistance: Double = when (voxelType) {
                                PROXIMITY -> {
                                    penetrationIntoVoxelSignedDistance
                                }
                                SURFACE -> {
                                    penetrationIntoVoxelSignedDistance - 1
                                }
                                INTERIOR -> {
                                    penetrationIntoVoxelSignedDistance - 2
                                }
                                else -> 0.0
                            }

                            if (signedDistance > 0) {
                                // No collision
                                return@TestNormal
                            }

                            minNormal.set(normalX, normalY, normalZ)
                            collisionPos.set(pointPosInBody0Coordinates).add(
                                penetrationIntoVoxelSignedDistance * normalX,
                                penetrationIntoVoxelSignedDistance * normalY,
                                penetrationIntoVoxelSignedDistance * normalZ
                            )

                            if (abs(signedDistance) > .01) {
                                println("error")
                            }

                            val body1CollisionPoint =
                                body1Transform.invTransform(
                                    body0Transform.transform(Vector3d(pointPosInBody0Coordinates))
                                )
                            val collisionNormal = body0Transform.rotate(Vector3d(minNormal)).mul(-1.0)

                            val body0CollisionPoint = Vector3d(collisionPos)

                            val collisionPair =
                                CollisionPair(body0CollisionPoint, body1CollisionPoint, collisionNormal)
                            collisionPairs.add(collisionPair)
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
