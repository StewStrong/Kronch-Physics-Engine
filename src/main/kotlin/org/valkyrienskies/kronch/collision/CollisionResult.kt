package org.valkyrienskies.kronch.collision

import java.util.stream.Stream

/**
 * Stores the collision points between two collision shapes.
 */
class CollisionResult(var _colliding: Boolean, var _collisionPoints: Collection<CollisionPairc>) : CollisionResultc {
    override val colliding: Boolean
        get() = _colliding
    override val collisionPoints: Stream<CollisionPairc>
        get() = run {
            if (!colliding) throw NotCollidingException("Cannot get collisionPoints because we are not colliding!")
            return _collisionPoints.stream()
        }
}
