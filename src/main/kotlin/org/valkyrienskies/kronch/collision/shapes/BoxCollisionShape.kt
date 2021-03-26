package org.valkyrienskies.kronch.collision.shapes

import org.joml.Vector3dc
import org.joml.primitives.AABBd
import org.joml.primitives.AABBdc
import org.valkyrienskies.kronch.Pose
import org.valkyrienskies.kronch.collision.CollisionShape
import org.valkyrienskies.kronch.collision.colliders.CollisionRange
import kotlin.math.max
import kotlin.math.min

data class BoxCollisionShape(val aabb: AABBdc) : CollisionShape {
    override fun getBoundingBox(output: AABBd): AABBd = output.set(aabb)

    companion object {
        fun projectBoxAlongAxis(
            box: BoxCollisionShape, pose: Pose, axis: Vector3dc, output: CollisionRange
        ): CollisionRange {
            output.min = Double.POSITIVE_INFINITY
            output.max = Double.NEGATIVE_INFINITY
            val boxPointsList = box.aabb.boxPoints()
            for (boxPoint in boxPointsList) {
                pose.transform(boxPoint)
                val alongAxis = axis.dot(boxPoint)
                output.min = min(output.min, alongAxis)
                output.max = max(output.max, alongAxis)
            }
            return output
        }
    }
}
