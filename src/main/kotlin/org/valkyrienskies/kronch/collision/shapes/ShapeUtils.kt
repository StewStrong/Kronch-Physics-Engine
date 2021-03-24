package org.valkyrienskies.kronch.collision.shapes

import org.joml.Vector3d
import org.joml.primitives.AABBdc

/**
 * Return a view of the points in this box
 *
 * The returned [Vector3d], [temp], is mutated whenever the list accessed, take care.
 */
fun AABBdc.boxPoints(temp: Vector3d = Vector3d()): List<Vector3d> = object : AbstractList<Vector3d>() {
    override val size: Int = 8

    override fun get(index: Int) = when (index) {
        0 -> temp.set(minX(), minY(), minZ())
        1 -> temp.set(minX(), minY(), maxZ())
        2 -> temp.set(minX(), maxY(), minZ())
        3 -> temp.set(minX(), maxY(), maxZ())
        4 -> temp.set(maxX(), minY(), minZ())
        5 -> temp.set(maxX(), minY(), maxZ())
        6 -> temp.set(maxX(), maxY(), minZ())
        7 -> temp.set(maxX(), maxY(), maxZ())
        else -> throw IndexOutOfBoundsException("Index: $index")
    }
}
