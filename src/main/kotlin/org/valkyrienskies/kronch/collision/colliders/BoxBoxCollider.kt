package org.valkyrienskies.kronch.collision.colliders

import org.joml.Vector3d
import org.joml.primitives.AABBdc
import org.valkyrienskies.kronch.Pose
import org.valkyrienskies.kronch.collision.CollisionPair
import org.valkyrienskies.kronch.collision.CollisionPairc
import org.valkyrienskies.kronch.collision.CollisionResult
import org.valkyrienskies.kronch.collision.CollisionResultc
import org.valkyrienskies.kronch.collision.NotCollidingException
import org.valkyrienskies.kronch.collision.shapes.BoxCollisionShape
import kotlin.math.abs
import kotlin.math.min

object BoxBoxCollider : Collider<BoxCollisionShape, BoxCollisionShape> {
    override fun computeCollisionBetweenShapes(
        firstBodyCollisionShape: BoxCollisionShape, firstBodyPose: Pose, secondBodyCollisionShape: BoxCollisionShape,
        secondBodyPose: Pose
    ): CollisionResultc {
        val collisionPairs: MutableList<CollisionPairc> = ArrayList()

        collideCubes(
            firstBodyCollisionShape, firstBodyPose, secondBodyCollisionShape, secondBodyPose
        ) { firstCollisionPoint: Vector3d, secondCollisionPoint: Vector3d, collisionNormal: Vector3d ->
            collisionPairs.add(CollisionPair(firstCollisionPoint, secondCollisionPoint, collisionNormal))
        }

        collideCubes(
            secondBodyCollisionShape, secondBodyPose, firstBodyCollisionShape, firstBodyPose
        ) { firstCollisionPoint: Vector3d, secondCollisionPoint: Vector3d, collisionNormal: Vector3d ->
            collisionPairs.add(CollisionPair(secondCollisionPoint, firstCollisionPoint, collisionNormal.mul(-1.0)))
        }

        return CollisionResult(collisionPairs)
    }

    private fun collideCubes(
        firstBodyCollisionShape: BoxCollisionShape, firstBodyPose: Pose, secondBodyCollisionShape: BoxCollisionShape,
        secondBodyPose: Pose, function: (Vector3d, Vector3d, Vector3d) -> Unit
    ) {
        for (firstBoxVertex in firstBodyCollisionShape.boxPointsIterator()) {
            firstBodyPose.transform(firstBoxVertex)
            secondBodyPose.invTransform(firstBoxVertex)
            // Now [firstBoxVertex] is in the local coordinates of [secondBodyCollisionShape]
            if (secondBodyCollisionShape.aabb.containsPoint(firstBoxVertex)) {
                // Collision found
                val collisionNormal = Vector3d()
                val collisionDepth = pushPointOutOfAABB(firstBoxVertex, secondBodyCollisionShape.aabb, collisionNormal)

                val firstCollisionPoint = Vector3d(firstBoxVertex)
                val secondCollisionPoint = Vector3d(firstBoxVertex).add(
                    collisionNormal.x() * collisionDepth, collisionNormal.y() * collisionDepth,
                    collisionNormal.z() * collisionDepth
                )

                secondBodyPose.transform(firstCollisionPoint)
                secondBodyPose.transform(secondCollisionPoint)
                secondBodyPose.rotate(collisionNormal)

                function(firstCollisionPoint, secondCollisionPoint, collisionNormal)
            }
        }
    }

    private fun pushPointOutOfAABB(point: Vector3d, aabb: AABBdc, outputNormal: Vector3d): Double {
        // Sanity check
        if (!aabb.containsPoint(point)) throw NotCollidingException("Point $point is not colliding with $aabb")

        val pushPosX = abs(aabb.maxX() - point.x())
        val pushNegX = abs(aabb.minX() - point.x())
        val pushPosY = abs(aabb.maxY() - point.y())
        val pushNegY = abs(aabb.minY() - point.y())
        val pushPosZ = abs(aabb.maxZ() - point.z())
        val pushNegZ = abs(aabb.minZ() - point.z())

        val minPushMag =
            minOf6(pushPosX, pushNegX, pushPosY, pushNegY, pushPosZ, pushNegZ)

        when (minPushMag) {
            pushPosX -> {
                outputNormal.set(1.0, 0.0, 0.0)
            }
            pushNegX -> {
                outputNormal.set(-1.0, 0.0, 0.0)
            }
            pushPosY -> {
                outputNormal.set(0.0, 1.0, 0.0)
            }
            pushNegY -> {
                outputNormal.set(0.0, -1.0, 0.0)
            }
            pushPosZ -> {
                outputNormal.set(0.0, 0.0, 1.0)
            }
            pushNegZ -> {
                outputNormal.set(0.0, 0.0, -1.0)
            }
        }
        return minPushMag
    }

    private fun minOf6(num0: Double, num1: Double, num2: Double, num3: Double, num4: Double, num5: Double): Double {
        return min(num0, min(num1, min(num2, min(num3, min(num4, num5)))))
    }
}
