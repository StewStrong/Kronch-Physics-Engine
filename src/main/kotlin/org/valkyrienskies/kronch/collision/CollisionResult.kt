package org.valkyrienskies.kronch.collision

/**
 * Stores the collision points between two collision shapes.
 */
class CollisionResult(var _collisionPoints: Collection<CollisionPairc>) :
    CollisionResultc {

    override val colliding: Boolean
        get() = run {
            _collisionPoints.isNotEmpty()
        }

    override val collisionPoints: Collection<CollisionPairc>
        get() = run {
            if (!colliding) throw NotCollidingException("Cannot get collisionPoints because we are not colliding!")
            return _collisionPoints
        }
}
