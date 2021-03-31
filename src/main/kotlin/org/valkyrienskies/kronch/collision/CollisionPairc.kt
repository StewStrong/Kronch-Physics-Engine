package org.valkyrienskies.kronch.collision

import org.joml.Vector3dc

/**
 * Immutable view of [CollisionPair].
 */
interface CollisionPairc {
    val positionInFirstBody: Vector3dc
    val positionInSecondBody: Vector3dc
    val normal: Vector3dc
    var used: Boolean
}
