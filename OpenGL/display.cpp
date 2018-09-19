#include "Display.h"
#include <shader_m.h>
#include <iostream>
Display::~Display() {}
Display::Display() {
}
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void processInput(GLFWwindow *window);
// camera
glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 3.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

int flag = 0;   
float deltaTime = 0.0f;	
float lastFrame = 0.0f;
float fov = 45.0f;
bool firstMouse = true;
Shader ourShader;
unsigned int VBO, VAO;
int showSub = 6;
std::vector<unsigned int>VAOsub(showSub);
std::vector<unsigned int>VBOsub(showSub);
std::vector<unsigned int>texturesub(showSub);
unsigned int texture1;
unsigned int SCR_WIDTH;
unsigned int SCR_HEIGHT;
float lastX = (float)SCR_WIDTH / 2.0;
float lastY = (float)SCR_HEIGHT / 2.0;

glm::vec3 cubePositions = glm::vec3(0.0f, 0.0f, 0.0f);

std::vector<float> vertices_face;

//input is left-bottom point
void Display::GenArray(int x, int y, int w, int h)
{
	//left-bottom is zero point
	float ratio_x = static_cast<float>(x - SCR_WIDTH / 2) / static_cast<float>(SCR_WIDTH / 2);
	float ratio_y = static_cast<float>(y - SCR_HEIGHT / 2) / static_cast<float>(SCR_HEIGHT / 2);
	float ratio_xr = static_cast<float>(x + w - SCR_WIDTH / 2) / static_cast<float>(SCR_WIDTH / 2);
	float ratio_yr = static_cast<float>(y + h - SCR_HEIGHT / 2) / static_cast<float>(SCR_HEIGHT / 2);
	vertices_face.push_back(ratio_x);
	vertices_face.push_back(ratio_y);
	vertices_face.push_back(0.0f);
	vertices_face.push_back(0.0f);
	vertices_face.push_back(1.0f);
	vertices_face.push_back(ratio_xr);
	vertices_face.push_back(ratio_y);
	vertices_face.push_back(0.0f);
	vertices_face.push_back(1.0f);
	vertices_face.push_back(1.0f);
	vertices_face.push_back(ratio_xr);
	vertices_face.push_back(ratio_yr);
	vertices_face.push_back(0.0f);
	vertices_face.push_back(1.0f);
	vertices_face.push_back(0.0f);
	vertices_face.push_back(ratio_xr);
	vertices_face.push_back(ratio_yr);
	vertices_face.push_back(0.0f);
	vertices_face.push_back(1.0f);
	vertices_face.push_back(0.0f);
	vertices_face.push_back(ratio_x);
	vertices_face.push_back(ratio_yr);
	vertices_face.push_back(0.0f);
	vertices_face.push_back(0.0f);
	vertices_face.push_back(0.0f);
	vertices_face.push_back(ratio_x);
	vertices_face.push_back(ratio_y);
	vertices_face.push_back(0.0f);
	vertices_face.push_back(0.0f);
	vertices_face.push_back(1.0f);
	// vertices_face[1] = ratio_y;
	// vertices_face[2] = 0.0f;
	// vertices_face[3] = 0.0f;
	// vertices_face[4] = 1.0f;
	// vertices_face[5] = ratio_xr;
	// vertices_face[6] = ratio_y;
	// vertices_face[7] = 0.0f;
	// vertices_face[8] = 1.0f;
	// vertices_face[9] = 1.0f;
	// vertices_face[10] = ratio_xr;
	// vertices_face[11] = ratio_yr;
	// vertices_face[12] = 0.0f;
	// vertices_face[13] = 1.0f;
	// vertices_face[14] = 0.0f;
	// vertices_face[15] = ratio_xr;
	// vertices_face[16] = ratio_yr;
	// vertices_face[17] = 0.0f;
	// vertices_face[18] = 1.0f;
	// vertices_face[19] = 0.0f;
	// vertices_face[20] = ratio_x;
	// vertices_face[21] = ratio_yr;
	// vertices_face[22] = 0.0f;
	// vertices_face[23] = 0.0f;
	// vertices_face[24] = 0.0f;
	// vertices_face[25] = ratio_x;
	// vertices_face[26] = ratio_y;
	// vertices_face[27] = 0.0f;
	// vertices_face[28] = 0.0f;
	// vertices_face[29] = 1.0f;


}

int Display::display_init(cv::Mat img)
{
	SCR_WIDTH = img.cols / 100 * 100 + 1300;
	SCR_HEIGHT = img.rows / 100 * 100;
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // uncomment this statement to fix compilation on OS X
#endif
	window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "show", NULL, NULL);

	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	ourShader.Shader1("E:/data/system_data/7.3.camera.vs", "E:/data/system_data/7.3.camera.fs");

	float width_ratio = (float)(img.cols - SCR_WIDTH / 2) / (float)(SCR_WIDTH / 2);
	float height_ratio = (float)img.rows / (float)img.cols * (1.0f + width_ratio) - 1.0f;
	float vertices[] = {
		-1.0f, 0.0f - height_ratio, -0.0f,  0.0f, 1.0f,
		width_ratio, 0.0f - height_ratio, -0.0f,  1.0f, 1.0f,
		width_ratio,  1.0f, -0.0f,  1.0f, 0.0f,
		width_ratio,  1.0f, -0.0f,  1.0f, 0.0f,
		-1.0f, 1.0f, -0.0f,  0.0f, 0.0f,
		-1.0f, 0.0f - height_ratio, -0.0f,  0.0f, 1.0f
	};
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);


	GenArray(SCR_WIDTH - 1250, SCR_HEIGHT - 650, 600, 600);
	GenArray(SCR_WIDTH - 650, SCR_HEIGHT - 650, 600, 600);
	GenArray(SCR_WIDTH - 1250, SCR_HEIGHT - 1250, 600, 600);
	GenArray(SCR_WIDTH - 650, SCR_HEIGHT - 1250, 600, 600);
	GenArray(SCR_WIDTH - 1250, SCR_HEIGHT - 1850, 600, 600);
	GenArray(SCR_WIDTH - 650, SCR_HEIGHT - 1850, 600, 600);
	GenArray(SCR_WIDTH - 1250, SCR_HEIGHT - 2450, 600, 600);
	GenArray(SCR_WIDTH - 650, SCR_HEIGHT - 2450, 600, 600);
	//float *vertices1 = GenArray(SCR_WIDTH - 1250, SCR_HEIGHT - 3050, 600, 600);
	//float *vertices1 = GenArray(SCR_WIDTH - 650, SCR_HEIGHT - 3050, 600, 600);
	float array_face[8 * 30];
	for (int i = 0; i < showSub * 30; i++)
	{
		array_face[i] = vertices_face[i];
	}
	for (int i = 0; i < showSub; i++)
	{
		glGenVertexArrays(1, &VAOsub[i]);
		glGenBuffers(1, &VBOsub[i]);
		glBindVertexArray(VAOsub[i]);
		glBindBuffer(GL_ARRAY_BUFFER, VBOsub[i]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), array_face + 30 * i, GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);
	}

	glGenTextures(1, &texture1);
	glBindTexture(GL_TEXTURE_2D, texture1);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	for (int i = 0; i < showSub; i++)
	{
		glGenTextures(1, &texturesub[i]);
		glBindTexture(GL_TEXTURE_2D, texturesub[i]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	}
	ourShader.use();
	ourShader.setInt("texture1", 0);
	return 0;
}


int Display::display(cv::Mat img, std::vector<cv::Mat> NeedToShow)
{
	glBindTexture(GL_TEXTURE_2D, texture1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.cols, img.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, img.data);
	for (int i = 0; i < NeedToShow.size(); i++)
	{
		cv::Mat temp;
		NeedToShow[i].copyTo(temp);
		cv::resize(temp, temp, cv::Size(600, 600));
		glBindTexture(GL_TEXTURE_2D, texturesub[i]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, temp.cols, temp.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, temp.data);

	}

	// -----------
	if (!glfwWindowShouldClose(window))
	{
		processInput(window);
		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glBindVertexArray(VAO);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, texture1);
		ourShader.use();
		glm::mat4 projection = glm::perspective(glm::radians(fov), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
		ourShader.setMat4("projection", projection);
		glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
		ourShader.setMat4("view", view);
		glm::mat4 model;
		ourShader.setMat4("model", model);
		glDrawArrays(GL_TRIANGLES, 0, 6);

		for (int i = 0; i < NeedToShow.size(); i++)
		{
			glBindVertexArray(VAOsub[i]);
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, texturesub[i]);
			ourShader.use();
			glm::mat4 projection1;
			ourShader.setMat4("projection", projection1);
			glm::mat4 view1;
			ourShader.setMat4("view", view1);
			glm::mat4 model1;
			ourShader.setMat4("model", model1);
			glDrawArrays(GL_TRIANGLES, 0, 6);
		}


		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	return 0;
}

void processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (flag == 1)
	{
		if (firstMouse)
		{
			lastX = xpos;
			lastY = ypos;
			firstMouse = false;
		}
		float xoffset = xpos - lastX;
		float yoffset = lastY - ypos;
		lastX = xpos;
		lastY = ypos;

		float x_delta = xoffset / sqrt((xoffset*xoffset + yoffset*yoffset));
		float y_delta = yoffset / sqrt((xoffset*xoffset + yoffset*yoffset));

		float cameraSpeed_x = 0.006 * x_delta * sqrt(fov);
		float cameraSpeed_y = 0.006 * y_delta * sqrt(fov);

		if (xoffset < 0)   //����
		{
			cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * std::fabs(cameraSpeed_x);
		}
		if (xoffset > 0)
		{
			cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * std::fabs(cameraSpeed_x);
		}
		if (yoffset < 0)
		{
			cameraPos += cameraUp * std::fabs(cameraSpeed_y);
		}
		if (yoffset > 0)
		{
			cameraPos -= cameraUp * std::fabs(cameraSpeed_y);
		}
	}

}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	if (fov >= 1.0f && fov <= 45.0f)
		fov -= yoffset * 1.2;
	if (fov <= 1.0f)
		fov = 1.0f;
	if (fov >= 45.0f)
		fov = 45.0f;
}
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (action == GLFW_PRESS) switch (button)
	{
	case GLFW_MOUSE_BUTTON_LEFT:
		flag = 1;
		break;
	case GLFW_MOUSE_BUTTON_MIDDLE:
		break;
	case GLFW_MOUSE_BUTTON_RIGHT:
		break;
	default:
		return;
	}
	if (action == GLFW_RELEASE) switch (button)
	{
	case GLFW_MOUSE_BUTTON_LEFT:
		flag = 0;
		break;
	case GLFW_MOUSE_BUTTON_MIDDLE:
		break;
	case GLFW_MOUSE_BUTTON_RIGHT:
		break;
	default:
		return;
	}
	return;
}
