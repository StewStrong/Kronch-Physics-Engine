package org.valkyrienskies.kronch.collision

import org.joml.primitives.AABBd

/**
 * The shape of an object in the physics engine.
 */
interface CollisionShape {
    fun getBoundingBox(output: AABBd): AABBd
}
