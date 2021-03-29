package org.valkyrienskies.kronch.collision.shapes

import org.joml.Vector3i
import org.joml.Vector3ic
import org.valkyrienskies.kronch.collision.shapes.VoxelShape.CollisionVoxelType.AIR
import org.valkyrienskies.kronch.collision.shapes.VoxelShape.CollisionVoxelType.INTERIOR
import org.valkyrienskies.kronch.collision.shapes.VoxelShape.CollisionVoxelType.PROXIMITY
import org.valkyrienskies.kronch.collision.shapes.VoxelShape.CollisionVoxelType.SURFACE
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

    private fun setVoxelType(pos: Vector3ic, newVoxelType: VoxelType): Boolean =
        setVoxelType(pos.x(), pos.y(), pos.z(), newVoxelType)

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
        forEachNeighbor(posX, posY, posZ) { neighborX: Int, neighborY: Int, neighborZ: Int ->
            run {
                if (!isPosInGrid(neighborX, neighborY, neighborZ)) return@run // Skip
                val neighborDataIndex = toIndex(neighborX, neighborY, neighborZ)
                val neighborOldData = grid[neighborDataIndex]

                val neighborOldCount =
                    neighborOldData.toInt() shr 1 // Convert to int because kotlin shifts dont work on bytes :(

                val neighborNewCount: Int = if (newVoxelType == FULL) {
                    neighborOldCount + 1
                } else {
                    neighborOldCount - 1
                }

                val neighborNewData = (neighborOldData and 0b00000001.toByte()) or (neighborNewCount shl 1).toByte()
                grid[neighborDataIndex] = neighborNewData
            }
        }

        return true // Something changed
    }

    private inline fun forEachNeighbor(
        posX: Int, posY: Int, posZ: Int, function: (neighborX: Int, neighborY: Int, neighborZ: Int) -> Unit
    ) {
        forEachNormal { normalX, normalY, normalZ ->
            function(posX + normalX, posY + normalY, posZ + normalZ)
        }
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

    fun getCollisionVoxelType(pos: Vector3ic): CollisionVoxelType = getCollisionVoxelType(pos.x(), pos.y(), pos.z())

    fun getCollisionVoxelType(posX: Int, posY: Int, posZ: Int): CollisionVoxelType {
        if (!isPosInGrid(posX, posY, posZ)) return AIR
        val index = toIndex(posX, posY, posZ)
        val data = grid[index]
        val isVoxelSet = (data and 0b00000001.toByte()) == 0b00000001.toByte()
        val neighborVoxelCount = data.toInt() shr 1
        return if (!isVoxelSet) {
            if (neighborVoxelCount == 0) {
                AIR
            } else {
                PROXIMITY
            }
        } else {
            if (neighborVoxelCount != 6) {
                SURFACE
            } else {
                INTERIOR
            }
        }
    }

    inline fun forEachAllowedNormal(
        posX: Int, posY: Int, posZ: Int, function: (normalX: Double, normalY: Double, normalZ: Double) -> Unit
    ) {
        val voxelType = getCollisionVoxelType(posX, posY, posZ)
        if (voxelType == AIR) {
            return
        }
        forEachNormal { normalX: Int, normalY: Int, normalZ: Int ->
            if (getCollisionVoxelType(posX - normalX, posY - normalY, posZ - normalZ).canPush(voxelType)) {
                function(normalX.toDouble(), normalY.toDouble(), normalZ.toDouble())
            }
        }
    }

    inline fun forEachNormal(function: (normalX: Int, normalY: Int, normalZ: Int) -> Unit) {
        for (i in 0 until 6) {
            var normalX = 0
            var normalY = 0
            var normalZ = 0
            when (i) {
                0 -> normalX = 1
                1 -> normalX = -1
                2 -> normalY = 1
                3 -> normalY = -1
                4 -> normalZ = 1
                5 -> normalZ = -1
            }
            function(normalX, normalY, normalZ)
        }
        // Equivalent to the following, but much smaller bytecode
        /*
        function(-1, 0, 0)
        function(0, 1, 0)
        function(0, -1, 0)
        function(0, 0, 1)
        function(0, 0, -1)
         */
    }

    fun getSurfaceVoxels(): Collection<Vector3ic> {
        val surfaceVoxels = ArrayList<Vector3ic>()
        for (voxel in voxels) {
            if (getCollisionVoxelType(voxel) == SURFACE) {
                surfaceVoxels.add(voxel)
            }
        }
        return surfaceVoxels
    }

    enum class VoxelType {
        FULL, EMPTY
    }

    enum class CollisionVoxelType(val sortIndex: Int) {
        AIR(0), PROXIMITY(1), SURFACE(2), INTERIOR(3);

        fun canPush(otherType: CollisionVoxelType): Boolean {
            return sortIndex > otherType.sortIndex
        }
    }
}
