package org.valkyrienskies.kronch.collision

import org.joml.Vector3dc

/**
 * Immutable view of [CollisionPair].
 */
interface CollisionPairc {
    val positionInFirstBody: Vector3dc
    val positionInSecondBody: Vector3dc
    val normal: Vector3dc

    /**
     * If depth <= 0 then there is no collision.
     */
    val depth: Double
        get() = normal.dot(
            positionInFirstBody.x() - positionInSecondBody.x(),
            positionInFirstBody.y() - positionInSecondBody.y(),
            positionInFirstBody.z() - positionInSecondBody.z()
        )
}
