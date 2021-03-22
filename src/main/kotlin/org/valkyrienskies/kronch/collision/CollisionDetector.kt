package org.valkyrienskies.kronch.collision

import org.valkyrienskies.kronch.Pose

/**
 * Computes the collision points between two collision shapes.
 */
interface CollisionDetector {
    fun computeCollisionBetweenShapes(
        firstBodyCollisionShape: CollisionShape, firstBodyPose: Pose, secondBodyCollisionShape: CollisionShape,
        secondBodyPose: Pose
    ): CollisionResultc
}
