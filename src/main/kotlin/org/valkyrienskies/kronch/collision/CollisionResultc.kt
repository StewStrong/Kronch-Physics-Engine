package org.valkyrienskies.kronch.collision

import java.util.stream.Stream

/**
 * Immutable view of [CollisionResult].
 */
interface CollisionResultc {
    val colliding: Boolean
    val collisionPoints: Stream<CollisionPairc>
}
