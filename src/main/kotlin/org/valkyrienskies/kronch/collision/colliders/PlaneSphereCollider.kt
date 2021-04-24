package org.valkyrienskies.kronch.collision.colliders

import org.joml.Vector3d
import org.valkyrienskies.kronch.Pose
import org.valkyrienskies.kronch.collision.CollisionPair
import org.valkyrienskies.kronch.collision.CollisionResult
import org.valkyrienskies.kronch.collision.CollisionResultc
import org.valkyrienskies.kronch.collision.shapes.PlaneShape
import org.valkyrienskies.kronch.collision.shapes.SphereShape

object PlaneSphereCollider : Collider<PlaneShape, SphereShape> {
    override fun computeCollisionBetweenShapes(
        body0Shape: PlaneShape, body0Transform: Pose, body1Shape: SphereShape, body1Transform: Pose
    ): CollisionResultc? {
        val planeNormal = body0Transform.rotate(Vector3d(body0Shape.normal))
        val spherePlanePosDif = Vector3d(body1Transform.p).sub(body0Transform.p)
        val dotProduct = planeNormal.dot(spherePlanePosDif)

        if (dotProduct < body1Shape.radius) {
            val collisionDepth = body1Shape.radius - dotProduct

            val body0CollisionPoint =
                body0Transform.invTransform(
                    Vector3d(body1Transform.p).fma(-body1Shape.radius + collisionDepth, planeNormal)
                )
            val body1CollisionPoint =
                body1Transform.invTransform(Vector3d(body1Transform.p).fma(-body1Shape.radius, planeNormal))

            return CollisionResult(
                listOf(
                    CollisionPair(
                        body0CollisionPoint, body1CollisionPoint, planeNormal.mul(1.0, Vector3d()), collisionDepth
                    )
                )
            )
        }

        return null
    }
}
