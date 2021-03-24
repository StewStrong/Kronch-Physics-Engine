package org.valkyrienskies.kronch.collision.shapes

import org.joml.Vector3d
import org.joml.primitives.AABBd
import org.joml.primitives.AABBdc
import org.valkyrienskies.kronch.collision.CollisionShape

data class BoxCollisionShape(val aabb: AABBdc) : CollisionShape {
    override fun getBoundingBox(output: AABBd): AABBd {
        return output.set(aabb)
    }

    fun boxPointsIterator(temp: Vector3d = Vector3d()): Iterator<Vector3d> {
        class BoxPointIterator : Iterator<Vector3d> {
            var index = 0

            override fun hasNext() = index < 8

            override fun next(): Vector3d {
                when (index) {
                    0 -> temp.set(aabb.minX(), aabb.minY(), aabb.minZ())
                    1 -> temp.set(aabb.minX(), aabb.minY(), aabb.maxZ())
                    2 -> temp.set(aabb.minX(), aabb.maxY(), aabb.minZ())
                    3 -> temp.set(aabb.minX(), aabb.maxY(), aabb.maxZ())
                    4 -> temp.set(aabb.maxX(), aabb.minY(), aabb.minZ())
                    5 -> temp.set(aabb.maxX(), aabb.minY(), aabb.maxZ())
                    6 -> temp.set(aabb.maxX(), aabb.maxY(), aabb.minZ())
                    7 -> temp.set(aabb.maxX(), aabb.maxY(), aabb.maxZ())
                }
                index++
                return temp
            }
        }
        return BoxPointIterator()
    }
}
