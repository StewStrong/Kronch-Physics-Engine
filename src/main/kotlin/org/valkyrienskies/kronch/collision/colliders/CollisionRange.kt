package org.valkyrienskies.kronch.collision.colliders

import kotlin.math.abs

data class CollisionRange(var min: Double, var max: Double) {
    companion object {
        fun create(): CollisionRange {
            return CollisionRange(0.0, 0.0)
        }

        fun computeCollisionResponse(
            collisionRange1: CollisionRange,
            collisionRange2: CollisionRange,
            collisionRange1Velocity: Double = 0.0
        ): Double {
            val pushLeft = -collisionRange1.max + collisionRange2.min
            val pushRight = -collisionRange1.min + collisionRange2.max

            var pushLeftWithRespectToVelocity = -collisionRange1.max + collisionRange2.min
            var pushRightWithRespectToVelocity = -collisionRange1.min + collisionRange2.max

            if (collisionRange1Velocity > 0) {
                pushLeftWithRespectToVelocity -= collisionRange1Velocity
            } else {
                pushRightWithRespectToVelocity -= collisionRange1Velocity
            }

            return if (pushRightWithRespectToVelocity <= 0 || pushLeftWithRespectToVelocity >= 0) {
                // Not overlapping
                0.0
            } else if (abs(pushLeft) > abs(pushRight)) {
                // Its more efficient to push [collisionRange1] left
                pushRightWithRespectToVelocity
            } else {
                // Its more efficient to push [collisionRange1] right
                pushLeftWithRespectToVelocity
            }
        }
    }
}
