package org.valkyrienskies.kronch.collision.shapes

import org.joml.primitives.AABBdc
import org.valkyrienskies.kronch.collision.colliders.CollisionRange
import kotlin.math.max
import kotlin.math.min

/**
 * Iterate over every point in a [AABBdc]
 */
fun AABBdc.forEachBoxPoint(function: (posX: Double, posY: Double, posZ: Double) -> Unit) {
    for (i in 0 until 8) {
        var posX = 0.0
        var posY = 0.0
        var posZ = 0.0
        when (i) {
            0 -> {
                posX = minX()
                posY = minY()
                posZ = minZ()
            }
            1 -> {
                posX = minX()
                posY = minY()
                posZ = maxZ()
            }
            2 -> {
                posX = minX()
                posY = maxY()
                posZ = minZ()
            }
            3 -> {
                posX = minX()
                posY = maxY()
                posZ = maxZ()
            }
            4 -> {
                posX = maxX()
                posY = minY()
                posZ = minZ()
            }
            5 -> {
                posX = maxX()
                posY = minY()
                posZ = maxZ()
            }
            6 -> {
                posX = maxX()
                posY = maxY()
                posZ = minZ()
            }
            7 -> {
                posX = maxX()
                posY = maxY()
                posZ = maxZ()
            }
        }
        function(posX, posY, posZ)
    }
}

fun getProjectionAlongAxis(
    aabb: AABBdc, normalX: Double, normalY: Double, normalZ: Double, output: CollisionRange
): CollisionRange {
    var minProjection = Double.POSITIVE_INFINITY
    var maxProjection = Double.NEGATIVE_INFINITY

    aabb.forEachBoxPoint { posX, posY, posZ ->
        val projection = dotProduct(normalX, normalY, normalZ, posX, posY, posZ)
        minProjection = min(minProjection, projection)
        maxProjection = max(maxProjection, projection)
    }

    output.min = minProjection
    output.max = maxProjection

    return output
}

fun dotProduct(aX: Double, aY: Double, aZ: Double, bX: Double, bY: Double, bZ: Double): Double {
    return aX * bX + aY * bY + aZ * bZ
}
