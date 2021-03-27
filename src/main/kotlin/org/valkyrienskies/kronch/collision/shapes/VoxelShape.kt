package org.valkyrienskies.kronch.collision.shapes

import org.joml.Vector3i
import org.joml.Vector3ic
import org.valkyrienskies.kronch.collision.shapes.VoxelShape.VoxelType.EMPTY
import org.valkyrienskies.kronch.collision.shapes.VoxelShape.VoxelType.FULL
import kotlin.experimental.and
import kotlin.experimental.or

class VoxelShape(
    val voxels: List<Vector3ic>, private val gridMin: Vector3ic = Vector3i(-20, -20, -20),
    private val gridMax: Vector3ic = Vector3i(20, 20, 20)
) : CollisionShape {

    private val grid: ByteArray

    init {
        val gridSize =
            (gridMax.x() - gridMin.x() + 1) * (gridMax.y() - gridMin.y() + 1) * (gridMax.z() - gridMin.z() + 1)
        grid = ByteArray(gridSize)

        voxels.forEach { setVoxelType(it, FULL) }
    }

    private fun isPosInGrid(posX: Int, posY: Int, posZ: Int): Boolean {
        return posX >= gridMin.x() && posX <= gridMax.x()
            && posY >= gridMin.y() && posY <= gridMax.y()
            && posZ >= gridMin.z() && posZ <= gridMax.z()
    }

    private fun toIndex(posX: Int, posY: Int, posZ: Int): Int {
        if (!isPosInGrid(posX, posY, posZ)) throw IllegalArgumentException(
            "Position $posX $posY $posZ is not in the voxel grid!"
        )
        val xLen = gridMax.x() - gridMin.x() + 1
        val yLen = gridMax.y() - gridMin.y() + 1
        return (posX - gridMin.x()) + xLen * (posY - gridMin.y()) + xLen * yLen * (posZ - gridMin.z())
    }

    private fun setVoxelType(pos: Vector3ic, newVoxelType: VoxelType): Boolean {
        return setVoxelType(pos.x(), pos.y(), pos.z(), newVoxelType)
    }

    private fun setVoxelType(posX: Int, posY: Int, posZ: Int, newVoxelType: VoxelType): Boolean {
        val prevVoxelType = getVoxelType(posX, posY, posZ)
        if (newVoxelType == prevVoxelType) return false // Nothing changed
        val index = toIndex(posX, posY, posZ)
        val data = grid[index]

        if (newVoxelType == FULL) {
            // Set bottom bit to 1
            grid[index] = data or 0b00000001.toByte()
        } else {
            // Set bottom bit to 0
            grid[index] = data and 0b11111110.toByte()
        }

        // Update neighbors
        for (xOffset in -1..1) {
            for (yOffset in -1..1) {
                for (zOffset in -1..1) {
                    if (xOffset == 0 && yOffset == 0 && zOffset == 0) continue // Skip
                    val neighborX = posX + xOffset
                    val neighborY = posY + yOffset
                    val neighborZ = posZ + zOffset
                    if (!isPosInGrid(neighborX, neighborY, neighborZ)) continue // Skip
                    val neighborDataIndex = toIndex(neighborX, neighborY, neighborZ)
                    val neighborOldData = grid[neighborDataIndex]

                    val neighborOldCount =
                        neighborOldData.toInt() shr 1 // Convert to int because kotlin shifts dont work on bytes :(

                    val neighborNewCount: Int = if (newVoxelType == FULL) {
                        neighborOldCount + 1
                    } else {
                        neighborOldCount - 1
                    }

                    val neighborNewData = (neighborOldData and 0b00000001) or (neighborNewCount shl 1).toByte()
                    grid[neighborDataIndex] = neighborNewData
                }
            }
        }

        return true // Something changed
    }

    private fun getVoxelType(posX: Int, posY: Int, posZ: Int): VoxelType {
        val index = toIndex(posX, posY, posZ)
        val data = grid[index]
        return if (data and 0x1.toByte() == 0x1.toByte()) {
            FULL
        } else {
            EMPTY
        }
    }

    enum class VoxelType {
        FULL, EMPTY
    }
}
