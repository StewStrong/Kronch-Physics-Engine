package org.valkyrienskies.kronch.collision.shapes

import org.joml.primitives.AABBd
import org.joml.primitives.AABBdc
import org.valkyrienskies.kronch.collision.CollisionShape

data class BoxCollisionShape(val aabb: AABBdc) : CollisionShape {
    override fun getBoundingBox(output: AABBd): AABBd = output.set(aabb)
}
