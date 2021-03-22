package org.valkyrienskies.kronch.collision.colliders

import org.valkyrienskies.kronch.Pose
import org.valkyrienskies.kronch.collision.CollisionResultc
import org.valkyrienskies.kronch.collision.CollisionShape

/**
 * Computes the collision points between two collision shapes.
 */
interface Collider<in FirstShapeType : CollisionShape, in SecondShapeType : CollisionShape> {
    fun computeCollisionBetweenShapes(
        firstBodyCollisionShape: FirstShapeType, firstBodyPose: Pose, secondBodyCollisionShape: SecondShapeType,
        secondBodyPose: Pose
    ): CollisionResultc
}
