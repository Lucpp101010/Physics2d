#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include "Draw.h"
#include "Simulation.h"

void onError(int code, const char* msg)
{
	std::cerr << "ERROR: " << code << " " << msg << std::endl;
}

int main()
{
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_SAMPLES, 4);
	GLFWwindow* window = glfwCreateWindow(1280, 1280, "Physics", NULL, NULL);
	glfwMakeContextCurrent(window);

	gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
	glViewport(0, 0, 1280, 1280);

	glfwSetErrorCallback(onError);

	glEnable(GL_MULTISAMPLE);

	//Scene scene;

	Simulation simulation;
	simulation.screenWidth = 1280;
	simulation.screenHeight = 1280;
	//simulation.aspectRatio = scene.screenWidth / scene.screenHeight;
	simulation.corner1 = { 0, 0 };
	simulation.corner2 = { 100, 100 };
	simulation.shader = Program::load("src/shader/shader");

	//scene.simulation.circles.push_back({ {50, 50}, 10 });
	//scene.simulation.circles.push_back({ {20, 10}, 10 });
	//scene.simulation.circles.push_back({ {20, 40}, 15 });
	//scene.simulation.circles.push_back({ {90, 80}, 20 });

	// border
	addShape(Shape({ {0, 1}, {100, 1}, {100, -101}, {0, -101} }, 1, { 0.2, 0.9, 0.1 }));
	addShape(Shape({ {1, 100}, {1, 0}, {-101, 0}, {-101, 100} }, 1, { 0.2, 0.9, 0.1 }));
	// bodies
	addShape(Shape({ {5, 20}, {5, -20}, {-5, -20}, {-5, 20} }, 1700.0 / 12.0, {0.5, 0.3, 0.6}));
	addShape(Shape({ {0, 5}, {5, -5}, {-5, -5} }, 80, { 0.9, 0.1, 0.1 }));
	addShape(Shape({ {10, 10}, {15, 0}, {10, -10}, {0, -15}, {-10, -10}, {-15, 0}, {-10, 10}, {0, 15} }, 500, {0.1, 0.2, 0.7}));

	simulation.rigidBodies.push_back({ { 0, 0 }, 0 });
	simulation.rigidBodies.back().fixed = true;
	simulation.rigidBodies.back().mass = 1e18;
	simulation.rigidBodies.push_back({ { 0, 200 }, 0 });
	simulation.rigidBodies.back().fixed = true;
	simulation.rigidBodies.back().mass = 1e18;
	simulation.rigidBodies.push_back({ { 0, 0 }, 1 });
	simulation.rigidBodies.back().fixed = true;
	simulation.rigidBodies.back().mass = 1e18;
	simulation.rigidBodies.push_back({ { 200, 0 }, 1 });
	simulation.rigidBodies.back().fixed = true;
	simulation.rigidBodies.back().mass = 1e18;

	//simulation.rigidBodies.back().fixed = true;
	simulation.rigidBodies.push_back({ {40, 40}, 2 });
	simulation.rigidBodies.back().angle = 1;
	simulation.rigidBodies.back().mass = 2;

	simulation.rigidBodies.push_back({ {50, 80}, 2 });
	simulation.rigidBodies.back().mass = 2;
	
	simulation.rigidBodies.push_back({ {80, 20}, 3 });
	simulation.rigidBodies.back().mass = 1;
	simulation.rigidBodies.push_back({ {90, 20}, 3 });
	simulation.rigidBodies.back().mass = 1;
	simulation.rigidBodies.push_back({ {80, 10}, 3 });
	simulation.rigidBodies.back().mass = 1;
	simulation.rigidBodies.push_back({ {90, 10}, 3 });
	simulation.rigidBodies.back().mass = 1;
	simulation.rigidBodies.push_back({ {80, 40}, 3 });
	simulation.rigidBodies.back().mass = 1;
	simulation.rigidBodies.push_back({ {80, 50}, 3 });
	simulation.rigidBodies.back().mass = 1;
	simulation.rigidBodies.push_back({ {80, 60}, 3 });
	simulation.rigidBodies.back().mass = 1;
	simulation.rigidBodies.push_back({ {80, 70}, 3 });
	simulation.rigidBodies.back().mass = 1;
	simulation.rigidBodies.push_back({ {80, 80}, 3 });
	simulation.rigidBodies.back().mass = 1;
	simulation.rigidBodies.push_back({ {80, 90}, 3 });
	simulation.rigidBodies.back().mass = 1;
	simulation.rigidBodies.push_back({ {70, 40}, 3 });
	simulation.rigidBodies.back().mass = 1;
	simulation.rigidBodies.push_back({ {70, 50}, 3 });
	simulation.rigidBodies.back().mass = 1;
	simulation.rigidBodies.push_back({ {70, 60}, 3 });
	simulation.rigidBodies.back().mass = 1;
	simulation.rigidBodies.push_back({ {70, 70}, 3 });
	simulation.rigidBodies.back().mass = 1;
	simulation.rigidBodies.push_back({ {70, 80}, 3 });
	simulation.rigidBodies.back().mass = 1;
	simulation.rigidBodies.push_back({ {70, 90}, 3 });
	simulation.rigidBodies.back().mass = 1;

	simulation.rigidBodies.push_back({ {20, 80}, 4 });
	simulation.rigidBodies.back().mass = 3;

	std::vector<RigidBody*> selected;
	Vec lastMousePos;

	double lastTime = glfwGetTime();

	while (!glfwWindowShouldClose(window))
	{
		double time = glfwGetTime();
		double dt = time - lastTime;
		lastTime = time;
		std::cout << "FPS: " << 1.0 / dt << std::endl;

		GLenum error = glGetError();
		if (error != GL_NO_ERROR)
		{
			std::cerr << error << std::endl;
		}

		glClearColor(0.1, 0.1, 0.1, 1);
		glClear(GL_COLOR_BUFFER_BIT);

		for (RigidBody& b : simulation.rigidBodies)
		{
			b.active = true;
		}

		double mouseX, mouseY;
		glfwGetCursorPos(window, &mouseX, &mouseY);
		Vec mousePos = simulation.fromScreenPoint({ mouseX, mouseY });

		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) != GLFW_PRESS)
		{
			selected = simulation.atPoint(mousePos);
		}
		else
		{
			Vec offset = mousePos - lastMousePos;
			for (RigidBody* b : selected)
			{
				b->pos += offset;
				b->active = false;
				b->v = offset / dt;
			}
		}

		simulation.simulate(0.3*dt);
		simulation.draw();

		lastMousePos = mousePos;

		glfwPollEvents();
		glfwSwapBuffers(window);
	}

	glfwDestroyWindow(window);
}