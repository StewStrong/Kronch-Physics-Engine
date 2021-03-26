package org.valkyrienskies.kronch.collision.colliders

import org.joml.Vector3d
import org.joml.Vector3dc
import org.joml.primitives.AABBdc
import org.valkyrienskies.kronch.Pose
import org.valkyrienskies.kronch.collision.CollisionPair
import org.valkyrienskies.kronch.collision.CollisionPairc
import org.valkyrienskies.kronch.collision.CollisionResult
import org.valkyrienskies.kronch.collision.CollisionResultc
import org.valkyrienskies.kronch.collision.NotCollidingException
import org.valkyrienskies.kronch.collision.shapes.BoxCollisionShape
import org.valkyrienskies.kronch.collision.shapes.boxPoints
import org.valkyrienskies.kronch.getQuatAxis0
import org.valkyrienskies.kronch.getQuatAxis1
import org.valkyrienskies.kronch.getQuatAxis2
import kotlin.math.abs
import kotlin.math.min

object BoxBoxCollider : Collider<BoxCollisionShape, BoxCollisionShape> {
    override fun computeCollisionBetweenShapes(
        firstBodyCollisionShape: BoxCollisionShape, firstBodyPose: Pose, secondBodyCollisionShape: BoxCollisionShape,
        secondBodyPose: Pose
    ): CollisionResultc {
        val collisionPairs: MutableList<CollisionPairc> = ArrayList()

        // Vertex collision
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

        // SAT Collision
        run {
            val normals: MutableList<Vector3dc?> = ArrayList()
            normals.add(getQuatAxis0(firstBodyPose.q))
            normals.add(getQuatAxis1(firstBodyPose.q))
            normals.add(getQuatAxis2(firstBodyPose.q))
            normals.add(getQuatAxis0(secondBodyPose.q))
            normals.add(getQuatAxis1(secondBodyPose.q))
            normals.add(getQuatAxis2(secondBodyPose.q))
            for (i in 0..2) {
                for (j in 3..5) {
                    val crossProduct = normals[i]!!.cross(normals[j], Vector3d())
                    if (crossProduct.lengthSquared() > 1e-6) {
                        crossProduct.normalize()
                        normals.add(crossProduct)
                    } else {
                        // Normal not valid
                        normals.add(null)
                    }
                }
            }

            val temp1 = CollisionRange.create()
            val temp2 = CollisionRange.create()

            var minResponse = Double.MAX_VALUE
            var minNormalIndex = -1
            var flipNormal = false

            for (i in 0 until normals.size) {
                val normal = normals[i] ?: continue // Skip null normals
                val firstBodyRange =
                    BoxCollisionShape.projectBoxAlongAxis(firstBodyCollisionShape, firstBodyPose, normal, temp1)
                val secondBodyRange =
                    BoxCollisionShape.projectBoxAlongAxis(secondBodyCollisionShape, secondBodyPose, normal, temp2)
                val response = CollisionRange.computeCollisionResponse(firstBodyRange, secondBodyRange)
                if (response == 0.0) {
                    // Not colliding
                    return CollisionResult(listOf())
                }
                if (abs(response) < minResponse) {
                    minResponse = abs(response)
                    flipNormal = response > 0
                    minNormalIndex = i
                }
            }

            if (minNormalIndex < 6) {
                // Face collision
            } else {
                // Edge collision
                // val body0EdgeNormal: Vector3dc = normals[(minNormalIndex - 5) / 3]!!
                // val body1EdgeNormal: Vector3dc = normals[((minNormalIndex - 5) % 3) + 3]!!
                val collisionNormal: Vector3dc = normals[minNormalIndex]!!

                val body0Support0 = Vector3d()
                val body0Support1 = Vector3d()
                val body1Support0 = Vector3d()
                val body1Support1 = Vector3d()

                setSupportVectors(
                    firstBodyCollisionShape, firstBodyPose, collisionNormal, flipNormal xor true, body0Support0,
                    body0Support1
                )
                setSupportVectors(
                    secondBodyCollisionShape, secondBodyPose, collisionNormal, flipNormal xor false, body1Support0,
                    body1Support1
                )

                val body0SupportDirection = body0Support1.sub(body0Support0, Vector3d())
                val body1SupportDirection = body1Support1.sub(body1Support0, Vector3d())

                // Collision between 2 edges is at the closest points on each line
                // Implementation based off of Bullet physics https://github.com/bulletphysics/bullet3/blob/afa4fb54505fd071103b8e2e8793c38fd40f6fb6/src/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp#L83-L107
                val p = body1Support0.sub(body0Support0, Vector3d())
                val uaub = body0SupportDirection.dot(body1SupportDirection)
                val q1 = body0SupportDirection.dot(p)
                val q2 = -body1SupportDirection.dot(p)
                val d = 1.0 / (1 - uaub * uaub)

                if (abs(d) > 1e4) {
                    // Give up
                    return CollisionResult(listOf())
                }

                val alpha = (q1 + uaub * q2) * d
                val beta = (uaub * q1 + q2) * d

                val body0CollisionPoint = Vector3d(body0Support0).fma(alpha, body0SupportDirection)
                val body1CollisionPoint = Vector3d(body1Support0).fma(beta, body1SupportDirection)

                return CollisionResult(listOf(CollisionPair(body0CollisionPoint, body1CollisionPoint, collisionNormal)))
            }
        }


        return CollisionResult(collisionPairs)
    }

    private fun setSupportVectors(
        boxCollisionShape: BoxCollisionShape, boxPose: Pose, collisionNormal: Vector3dc, invertNormal: Boolean,
        support0: Vector3d, support1: Vector3d
    ) {
        var smallestProjection = Double.MAX_VALUE

        for (box0Point in boxCollisionShape.aabb.boxPoints()) {
            boxPose.transform(box0Point)
            val alongAxis = if (invertNormal) -collisionNormal.dot(box0Point) else collisionNormal.dot(box0Point)
            smallestProjection = min(smallestProjection, alongAxis)
        }

        var setSupport0 = false

        for (box0Point in boxCollisionShape.aabb.boxPoints()) {
            boxPose.transform(box0Point)
            val alongAxis = if (invertNormal) -collisionNormal.dot(box0Point) else collisionNormal.dot(box0Point)
            if (equals(alongAxis, smallestProjection)) {
                if (!setSupport0) {
                    support0.set(box0Point)
                    setSupport0 = true
                } else {
                    support1.set(box0Point)
                }
            }
        }
    }

    fun equals(a: Double, b: Double, EPSILON: Double = 1e-10): Boolean {
        return abs(a - b) < EPSILON
    }

    private inline fun collideCubes(
        firstBodyCollisionShape: BoxCollisionShape, firstBodyPose: Pose, secondBodyCollisionShape: BoxCollisionShape,
        secondBodyPose: Pose, function: (Vector3d, Vector3d, Vector3d) -> Unit
    ) {
        for (firstBoxVertex in firstBodyCollisionShape.aabb.boxPoints()) {
            firstBodyPose.transform(firstBoxVertex)
            secondBodyPose.invTransform(firstBoxVertex)
            // Now [firstBoxVertex] is in the local coordinates of [secondBodyCollisionShape]
            if (secondBodyCollisionShape.aabb.containsPoint(firstBoxVertex)) {
                // Collision found
                val collisionNormal = Vector3d()
                val collisionDepth = pushPointOutOfAABB(firstBoxVertex, secondBodyCollisionShape.aabb, collisionNormal)

                val firstCollisionPoint = Vector3d(firstBoxVertex)
                val secondCollisionPoint = Vector3d(firstBoxVertex).fma(collisionDepth, collisionNormal)

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

        val minPushMag = minOf6(pushPosX, pushNegX, pushPosY, pushNegY, pushPosZ, pushNegZ)

        when (minPushMag) {
            pushPosX -> outputNormal.set(1.0, 0.0, 0.0)
            pushNegX -> outputNormal.set(-1.0, 0.0, 0.0)
            pushPosY -> outputNormal.set(0.0, 1.0, 0.0)
            pushNegY -> outputNormal.set(0.0, -1.0, 0.0)
            pushPosZ -> outputNormal.set(0.0, 0.0, 1.0)
            pushNegZ -> outputNormal.set(0.0, 0.0, -1.0)
        }

        return minPushMag
    }

    private fun minOf6(num0: Double, num1: Double, num2: Double, num3: Double, num4: Double, num5: Double): Double {
        return min(num0, min(num1, min(num2, min(num3, min(num4, num5)))))
    }
}
