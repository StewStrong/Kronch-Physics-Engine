package org.valkyrienskies.kronch.collision.colliders

import org.valkyrienskies.kronch.Pose
import org.valkyrienskies.kronch.collision.CollisionResultc
import org.valkyrienskies.kronch.collision.shapes.VoxelShape

object VoxelVoxelCollider : Collider<VoxelShape, VoxelShape> {
    override fun computeCollisionBetweenShapes(
        body0Shape: VoxelShape, body0Transform: Pose, body1Shape: VoxelShape, body1Transform: Pose
    ): CollisionResultc? {
        return null
    }
}
