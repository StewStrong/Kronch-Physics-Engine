package org.valkyrienskies.kronch.collision.colliders

import org.joml.Vector3d
import org.joml.Vector3dc
import org.valkyrienskies.kronch.Pose
import org.valkyrienskies.kronch.collision.CollisionResultc
import org.valkyrienskies.kronch.collision.shapes.BoxShape
import org.valkyrienskies.kronch.collision.shapes.SphereShape
import kotlin.math.max
import kotlin.math.min

object BoxSphereCollider : Collider<BoxShape, SphereShape> {
    override fun computeCollisionBetweenShapes(
        body0Shape: BoxShape, body0Transform: Pose, body1Shape: SphereShape, body1Transform: Pose
    ): CollisionResultc? {
        // Sphere position in body0 coordinates
        val sphereRelativePos: Vector3dc = body0Transform.invTransform(Vector3d(body1Transform.p))

        // The point on the box that is closest to the sphere
        val boxClosestPoint = Vector3d(sphereRelativePos)

        boxClosestPoint.x = min(body0Shape.size.x(), boxClosestPoint.x())
        boxClosestPoint.x = max(-body0Shape.size.x(), boxClosestPoint.x())
        boxClosestPoint.y = min(body0Shape.size.y(), boxClosestPoint.y())
        boxClosestPoint.y = max(-body0Shape.size.y(), boxClosestPoint.y())
        boxClosestPoint.z = min(body0Shape.size.z(), boxClosestPoint.z())
        boxClosestPoint.z = max(-body0Shape.size.z(), boxClosestPoint.z())

        val collisionNormal = Vector3d(sphereRelativePos).sub(boxClosestPoint)

        if (collisionNormal.lengthSquared() > body1Shape.radius * body1Shape.radius) {
            return null // No collision
        }
        


        return null
    }
}
