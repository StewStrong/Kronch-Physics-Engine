package org.valkyrienskies.kronch.collision

import org.joml.Vector3dc

/**
 * A [CollisionPair] describes how to resolve a collision using two points. Both of these points must be pushed apart along normal to resolve the collision.
 */
class CollisionPair(
    var _positionInFirstBody: Vector3dc, var _positionInSecondBody: Vector3dc, var _normal: Vector3dc
) : CollisionPairc {
    override val positionInFirstBody: Vector3dc
        get() = _positionInFirstBody
    override val positionInSecondBody: Vector3dc
        get() = _positionInSecondBody
    override val normal: Vector3dc
        get() = _normal
}
