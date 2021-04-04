package org.valkyrienskies.kronch.collision

/**
 * Immutable view of [CollisionResult].
 */
interface CollisionResultc {
    val colliding: Boolean
    val collisionPoints: Collection<CollisionPairc>
}
