package org.valkyrienskies.kronch.collision

import org.joml.Vector3dc

/**
 * A [CollisionPair] describes how to resolve a collision using two points. Both of these points must be pushed apart along normal to resolve the collision.
 */
class CollisionPair(
    override var positionInFirstBody: Vector3dc,
    override var positionInSecondBody: Vector3dc,
    override var normal: Vector3dc,
    override var used: Boolean = false
) : CollisionPairc
