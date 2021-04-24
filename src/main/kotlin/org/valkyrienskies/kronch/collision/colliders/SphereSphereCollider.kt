package org.valkyrienskies.kronch.collision.colliders

import org.joml.Vector3d
import org.valkyrienskies.kronch.Pose
import org.valkyrienskies.kronch.collision.CollisionPair
import org.valkyrienskies.kronch.collision.CollisionResult
import org.valkyrienskies.kronch.collision.CollisionResultc
import org.valkyrienskies.kronch.collision.shapes.SphereShape

object SphereSphereCollider : Collider<SphereShape, SphereShape> {
    override fun computeCollisionBetweenShapes(
        body0Shape: SphereShape, body0Transform: Pose, body1Shape: SphereShape, body1Transform: Pose
    ): CollisionResultc? {
        val positionDifference = Vector3d(body1Transform.p).sub(body0Transform.p)

        val totalRadius = body0Shape.radius + body1Shape.radius

        if (positionDifference.length() < 1e-20) return null // Too small

        if (positionDifference.length() < totalRadius) {
            val normal = Vector3d(positionDifference).normalize()
            val body0CollisionPoint =
                body0Transform.invTransform(Vector3d(body0Transform.p).fma(body0Shape.radius, normal))
            val body1CollisionPoint =
                body1Transform.invTransform(Vector3d(body1Transform.p).fma(-body1Shape.radius, normal))
            return CollisionResult(
                listOf(
                    CollisionPair(
                        body0CollisionPoint, body1CollisionPoint, normal.mul(1.0, Vector3d()),
                        totalRadius - positionDifference.length()
                    )
                )
            )
        }

        return null
    }
}
