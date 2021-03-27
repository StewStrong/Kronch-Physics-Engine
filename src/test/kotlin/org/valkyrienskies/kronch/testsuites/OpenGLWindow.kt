package org.valkyrienskies.kronch.testsuites

import org.joml.AxisAngle4d
import org.joml.Matrix4f
import org.joml.Matrix4x3f
import org.lwjgl.BufferUtils
import org.lwjgl.glfw.GLFW
import org.lwjgl.glfw.GLFWErrorCallback
import org.lwjgl.glfw.GLFWFramebufferSizeCallback
import org.lwjgl.glfw.GLFWKeyCallback
import org.lwjgl.opengl.GL
import org.lwjgl.opengl.GL11
import org.lwjgl.system.MemoryUtil
import org.valkyrienskies.kronch.PhysicsWorld
import org.valkyrienskies.kronch.collision.shapes.VoxelShape
import java.nio.FloatBuffer

/**
 * Creates a window that renders using OpenGL. Based off of https://github.com/LWJGL/lwjgl3-demos/blob/79e81f6f3794c3dfd32ba0a31aefa69e56ad603b/src/org/lwjgl/demo/opengl/transform/LwjglDemo.java.
 */
class OpenGLWindow {
    var errorCallback: GLFWErrorCallback? = null
    var keyCallback: GLFWKeyCallback? = null
    var fbCallback: GLFWFramebufferSizeCallback? = null

    var window: Long = 0
    var width = 400
    var height = 300

    // JOML matrices
    var projMatrix = Matrix4f()
    var viewMatrix = Matrix4x3f()

    // FloatBuffer for transferring matrices to OpenGL
    private var floatBuffer: FloatBuffer = BufferUtils.createFloatBuffer(16)

    private val physicsWorld = PhysicsWorld()

    fun run() {
        try {
            init()
            loop()
            GLFW.glfwDestroyWindow(window)
            keyCallback!!.free()
        } finally {
            GLFW.glfwTerminate()
            errorCallback!!.free()
        }
    }

    private fun init() {
        GLFW.glfwSetErrorCallback(GLFWErrorCallback.createPrint(System.err).also { errorCallback = it })
        check(GLFW.glfwInit()) { "Unable to initialize GLFW" }

        // Configure our window
        GLFW.glfwDefaultWindowHints()
        GLFW.glfwWindowHint(GLFW.GLFW_VISIBLE, GLFW.GLFW_FALSE)
        GLFW.glfwWindowHint(GLFW.GLFW_RESIZABLE, GLFW.GLFW_TRUE)
        window = GLFW.glfwCreateWindow(width, height, "Hello World!", MemoryUtil.NULL, MemoryUtil.NULL)
        if (window == MemoryUtil.NULL) throw RuntimeException("Failed to create the GLFW window")
        GLFW.glfwSetKeyCallback(
            window,
            object : GLFWKeyCallback() {
                override fun invoke(window: Long, key: Int, scancode: Int, action: Int, mods: Int) {
                    if (key == GLFW.GLFW_KEY_ESCAPE && action == GLFW.GLFW_RELEASE) GLFW.glfwSetWindowShouldClose(
                        window, true
                    )
                    if (key == GLFW.GLFW_KEY_UP) {
                        for (body in physicsWorld.bodies) {
                            body.vel.add(0.0, 10.0, 0.0)
                        }
                    }
                }
            }.also { keyCallback = it }
        )
        GLFW.glfwSetFramebufferSizeCallback(
            window,
            object : GLFWFramebufferSizeCallback() {
                override fun invoke(window: Long, w: Int, h: Int) {
                    if (w > 0 && h > 0) {
                        width = w
                        height = h
                    }
                }
            }.also { fbCallback = it }
        )
        val vidmode = GLFW.glfwGetVideoMode(GLFW.glfwGetPrimaryMonitor())
        GLFW.glfwSetWindowPos(window, (vidmode!!.width() - width) / 2, (vidmode.height() - height) / 2)
        GLFW.glfwMakeContextCurrent(window)
        GLFW.glfwSwapInterval(0)
        GLFW.glfwShowWindow(window)
    }

    private fun renderCube() {
        GL11.glBegin(GL11.GL_QUADS)
        GL11.glColor3f(0.0f, 0.0f, 0.2f)
        GL11.glVertex3f(0.5f, -0.5f, -0.5f)
        GL11.glVertex3f(-0.5f, -0.5f, -0.5f)
        GL11.glVertex3f(-0.5f, 0.5f, -0.5f)
        GL11.glVertex3f(0.5f, 0.5f, -0.5f)
        GL11.glColor3f(0.0f, 0.0f, 1.0f)
        GL11.glVertex3f(0.5f, -0.5f, 0.5f)
        GL11.glVertex3f(0.5f, 0.5f, 0.5f)
        GL11.glVertex3f(-0.5f, 0.5f, 0.5f)
        GL11.glVertex3f(-0.5f, -0.5f, 0.5f)
        GL11.glColor3f(1.0f, 0.0f, 0.0f)
        GL11.glVertex3f(0.5f, -0.5f, -0.5f)
        GL11.glVertex3f(0.5f, 0.5f, -0.5f)
        GL11.glVertex3f(0.5f, 0.5f, 0.5f)
        GL11.glVertex3f(0.5f, -0.5f, 0.5f)
        GL11.glColor3f(0.2f, 0.0f, 0.0f)
        GL11.glVertex3f(-0.5f, -0.5f, 0.5f)
        GL11.glVertex3f(-0.5f, 0.5f, 0.5f)
        GL11.glVertex3f(-0.5f, 0.5f, -0.5f)
        GL11.glVertex3f(-0.5f, -0.5f, -0.5f)
        GL11.glColor3f(0.0f, 1.0f, 0.0f)
        GL11.glVertex3f(0.5f, 0.5f, 0.5f)
        GL11.glVertex3f(0.5f, 0.5f, -0.5f)
        GL11.glVertex3f(-0.5f, 0.5f, -0.5f)
        GL11.glVertex3f(-0.5f, 0.5f, 0.5f)
        GL11.glColor3f(0.0f, 0.2f, 0.0f)
        GL11.glVertex3f(0.5f, -0.5f, -0.5f)
        GL11.glVertex3f(0.5f, -0.5f, 0.5f)
        GL11.glVertex3f(-0.5f, -0.5f, 0.5f)
        GL11.glVertex3f(-0.5f, -0.5f, -0.5f)
        GL11.glEnd()
    }

    // Renders a plane from (-.5, 0, -.5) to (.5, 0, .5)
    private fun renderPlane() {
        GL11.glBegin(GL11.GL_QUADS)
        GL11.glColor3f(0.0f, 0.4f, 0.4f)
        GL11.glVertex3f(0.5f, 0.0f, 0.5f)
        GL11.glVertex3f(0.5f, 0.0f, -0.5f)
        GL11.glVertex3f(-0.5f, 0.0f, -0.5f)
        GL11.glVertex3f(-0.5f, 0.0f, 0.5f)
        GL11.glEnd()
    }

    private fun loop() {
        GL.createCapabilities()

        // Set the clear color
        GL11.glClearColor(0.6f, 0.7f, 0.8f, 1.0f)
        // Enable depth testing
        GL11.glEnable(GL11.GL_DEPTH_TEST)
        GL11.glEnable(GL11.GL_CULL_FACE)

        // Remember the current time.
        val firstTime = System.nanoTime()
        var lastTime = System.nanoTime()
        while (!GLFW.glfwWindowShouldClose(window)) {
            // Build time difference between this and first time.
            val thisTime = System.nanoTime()
            val diff = (thisTime - lastTime) / 1E9f

            // Run physics
            physicsWorld.simulate(.001) // diff.toDouble())

            // Compute some rotation angle.

            // Make the viewport always fill the whole window.
            GL11.glViewport(0, 0, width, height)

            // Build the projection matrix. Watch out here for integer division
            // when computing the aspect ratio!
            projMatrix.setPerspective(Math.toRadians(40.0).toFloat(), width.toFloat() / height, 0.01f, 100.0f)
            GL11.glMatrixMode(GL11.GL_PROJECTION)
            GL11.glLoadMatrixf(projMatrix[floatBuffer])

            // Set lookat view matrix
            viewMatrix.setLookAt(0.0f, 4.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f)
            GL11.glMatrixMode(GL11.GL_MODELVIEW)
            GL11.glClear(GL11.GL_COLOR_BUFFER_BIT or GL11.GL_DEPTH_BUFFER_BIT)

            // Load the view matrix
            GL11.glLoadMatrixf(viewMatrix.get4x4(floatBuffer))

            // Render the ground plane
            run {
                GL11.glPushMatrix()
                GL11.glPushMatrix()
                // GL11.glTranslatef(0.0f, -1.0f, 0.0f)
                GL11.glScalef(10.0f, 10.0f, 10.0f)
                renderPlane()
                GL11.glPopMatrix()
            }

            // Render cubes
            for (body in physicsWorld.bodies) {
                GL11.glPushMatrix()
                GL11.glTranslatef(body.position.x().toFloat(), body.position.y().toFloat(), body.position.z().toFloat())
                val axisAngle = AxisAngle4d().set(body.quaternion)
                GL11.glRotatef(
                    Math.toDegrees(axisAngle.angle).toFloat(), axisAngle.x.toFloat(), axisAngle.y.toFloat(),
                    axisAngle.z.toFloat()
                )
                val bodyShape = body.shape
                if (bodyShape is VoxelShape) {
                    for (voxelPos in bodyShape.voxels) {
                        GL11.glPushMatrix()
                        GL11.glTranslatef(voxelPos.x().toFloat(), voxelPos.y().toFloat(), voxelPos.z().toFloat())
                        renderCube()
                        GL11.glPopMatrix()
                    }
                }
                GL11.glPopMatrix()
            }

            GL11.glPopMatrix()
            GLFW.glfwSwapBuffers(window)
            GLFW.glfwPollEvents()

            lastTime = thisTime
        }
    }
}
