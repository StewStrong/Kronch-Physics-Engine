package org.valkyrienskies.kronch.testmath

import org.joml.Vector3d
import org.junit.jupiter.api.Test
import org.valkyrienskies.kronch.Body
import org.valkyrienskies.kronch.Pose
import org.valkyrienskies.kronch.applyBodyPairCorrection
import kotlin.math.abs

internal class TestBodyCorrection {

    /**
     * This tests setting the velocity of "body" at point "collisionPointInGlobalCoordinates" to be 0 along "normal"
     */
    @Test
    fun testSingleBodyCorrection() {
        val body = Body(Pose())
        body.setBox(Vector3d(1.0)) // Set inertia to be that of a box

        body.vel.set(200000.0, -200000.0, 50000.0)
        body.omega.set(997100.0, -26600.0, 57890.0)

        val collisionPointInBodyCoordinates = Vector3d(.25, .25, .25)
        val collisionPointInGlobalCoordinates = body.pose.transform(Vector3d(collisionPointInBodyCoordinates))

        val normal = Vector3d(0.0, 1.0, 0.0)

        val velocityAtPoint = body.getVelocityAt(collisionPointInGlobalCoordinates)

        val velocityAlongNormal = normal.dot(velocityAtPoint)

        val deltaVelocity = normal.mul(-velocityAlongNormal, Vector3d())

        applyBodyPairCorrection(body, null, deltaVelocity, 0.0, 1.0, collisionPointInGlobalCoordinates, null, true)

        // val inverseMassAtPoint = body.getInverseMass(normal, collisionPointInGlobalCoordinates)
        // val p = deltaVelocity.div(inverseMassAtPoint, Vector3d())
        // body.applyCorrection(p, collisionPointInGlobalCoordinates, true)

        val newVelocityAtPoint = body.getVelocityAt(collisionPointInGlobalCoordinates)

        assert(abs(normal.dot(newVelocityAtPoint)) < 1e-4)
    }
}
