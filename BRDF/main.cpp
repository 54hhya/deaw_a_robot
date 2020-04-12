#define STB_IMAGE_IMPLEMENTATION
#include <glad/glad.h>
#include <GLFW/glfw3.h>
// GLM Mathemtics
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "shader.h"
#include "camera.h"
#include "stb_image.h"
#include <iostream>
#include <cmath>
#include <vector>
using  namespace std;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);

unsigned int loadTexture(const char *path);
unsigned int loadCubemap(vector<std::string> faces);
bool IK(double x, double y, double l1, double l2, double &theta1, double &theta2);
bool IK1(double x, double y, double l1, double l2, double &theta1, double &theta2);
bool IK3(double x, double y, double l1, double l2, double &theta1, double &theta2);

int op = 1;      //当前执行的操作
double angel1 = 0, angel2 = 0;
double px = 0, py = -0.9;
double larm1 = 0, larm2 = 0, lleg1 = 0, lleg2 = 0, rarm1 = 0, rarm2 = 0, rleg1 = 0, rleg2 = 0;   //初始角度
double lax = 0, lay = -0.9, rax = 0, ray = -0.9;  //用于左臂行走的代码
double rlx = 0, rly = -0.9, llx = 0, lly = -0.9;
int direction = 1;      //规定一下前进的方向
float walk=0.0;
int ss = 1;      //规定行走状态

// 设置
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
// camera
Camera camera(glm::vec3(0.0f, 0.0f, 10.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

// 光源的位置
glm::vec3 lightPos(1.0f, 1.0f, 5.0f);

int main()
{
	// glfw: initialize and configure
	// ------------------------------
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // uncomment this statement to fix compilation on OS X
#endif

	// glfw window creation
	// --------------------
	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);

	// tell GLFW to capture our mouse
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// glad: load all OpenGL function pointers
	// ---------------------------------------
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	// configure global opengl state
	// -----------------------------
	glEnable(GL_DEPTH_TEST);

	// build and compile our shader zprogram
	// ------------------------------------
	Shader lightingShader("E:/ex3/BRDF/colors.vs.txt", "E:/ex3/BRDF/colors.fs.txt");
	Shader lampShader("E:/ex3/BRDF/lamp.vs.txt", "E:/ex3/BRDF/lamp.fs.txt");
	Shader skyboxShader("E:/ex3/BRDF/skybox.vs.txt", "E:/ex3/BRDF/skybox.fs.txt");  //天空盒纹理

	//lightingShader.setVec3("albedo", 0.5f, 0.0f, 0.0f);
	lightingShader.use();
	lightingShader.setInt("albedoMap", 0);       //反照率(Albedo)纹理为每一个金属的纹素(Texel)（纹理像素）指定表面颜色或者基础反射率
	lightingShader.setFloat("ao", 1.0f);         //AO贴图为表面和周围潜在的几何图形指定了一个额外的阴影因子
	lightingShader.setFloat("metallic", 0.8);    //指定该纹素是不是金属质地的
	lightingShader.setFloat("roughness", 0.4);   //指定某个表面有多粗糙

	// set up vertex data (and buffer(s)) and configure vertex attributes
	// ------------------------------------------------------------------
	float vertices[] = {
		-0.1f, -0.1f, -0.1f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f,
		 0.1f, -0.1f, -0.1f,  0.0f,  0.0f, -1.0f, 1.0f, 0.0f,
		 0.1f,  0.1f, -0.1f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f,
		 0.1f,  0.1f, -0.1f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f,
		-0.1f,  0.1f, -0.1f,  0.0f,  0.0f, -1.0f, 0.0f, 1.0f,
		-0.1f, -0.1f, -0.1f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f,

		-0.1f, -0.1f,  0.1f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f,
		 0.1f, -0.1f,  0.1f,  0.0f,  0.0f,  1.0f, 1.0f, 0.0f,
		 0.1f,  0.1f,  0.1f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f,
		 0.1f,  0.1f,  0.1f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f,
		-0.1f,  0.1f,  0.1f,  0.0f,  0.0f,  1.0f, 0.0f, 1.0f,
		-0.1f, -0.1f,  0.1f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f,


		-0.1f,  0.1f,  0.1f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f,
		-0.1f,  0.1f, -0.1f, -1.0f,  0.0f,  0.0f, 1.0f, 1.0f,
		-0.1f, -0.1f, -0.1f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f,
		-0.1f, -0.1f, -0.1f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f,
		-0.1f, -0.1f,  0.1f, -1.0f,  0.0f,  0.0f, 0.0f, 0.0f,
		-0.1f,  0.1f,  0.1f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f,

		 0.1f,  0.1f,  0.1f,  1.0f,  0.0f,  0.0f, 1.0f, 0.0f,
		 0.1f,  0.1f, -0.1f,  1.0f,  0.0f,  0.0f, 1.0f, 1.0f,
		 0.1f, -0.1f, -0.1f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f,
		 0.1f, -0.1f, -0.1f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f,
		 0.1f, -0.1f,  0.1f,  1.0f,  0.0f,  0.0f, 0.0f, 0.0f,
		 0.1f,  0.1f,  0.1f,  1.0f,  0.0f,  0.0f, 1.0f, 0.0f,

		-0.1f, -0.1f, -0.1f,  0.0f, -1.0f,  0.0f, 0.0f, 1.0f,
		 0.1f, -0.1f, -0.1f,  0.0f, -1.0f,  0.0f, 1.0f, 1.0f,
		 0.1f, -0.1f,  0.1f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f,
		 0.1f, -0.1f,  0.1f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f,
		-0.1f, -0.1f,  0.1f,  0.0f, -1.0f,  0.0f, 0.0f, 0.0f,
		-0.1f, -0.1f, -0.1f,  0.0f, -1.0f,  0.0f, 0.0f, 1.0f,

		-0.1f,  0.1f, -0.1f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f,
		 0.1f,  0.1f, -0.1f,  0.0f,  1.0f,  0.0f, 1.0f, 1.0f,
		 0.1f,  0.1f,  0.1f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f,
		 0.1f,  0.1f,  0.1f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f,
		-0.1f,  0.1f,  0.1f,  0.0f,  1.0f,  0.0f, 0.0f, 0.0f,
		-0.1f,  0.1f, -0.1f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f
	};

	float skyboxVertices[] = {
		// positions          
		-1.0f,  1.0f, -1.0f,
		-1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		-1.0f,  1.0f, -1.0f,
		 1.0f,  1.0f, -1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		 1.0f, -1.0f,  1.0f
	};

	// first, configure the cube's VAO (and VBO)
	unsigned int VBO, cubeVAO;
	glGenVertexArrays(1, &cubeVAO);
	glGenBuffers(1, &VBO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	glBindVertexArray(cubeVAO);

	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	// normal attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);
	//纹理attribute
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
	glEnableVertexAttribArray(2);

	// second, configure the light's VAO (VBO stays the same; the vertices are the same for the light object which is also a 3D cube)
	unsigned int lightVAO;
	glGenVertexArrays(1, &lightVAO);
	glBindVertexArray(lightVAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	// note that we update the lamp's position attribute's stride to reflect the updated buffer data
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	// skybox VAO
	unsigned int skyboxVAO, skyboxVBO;
	glGenVertexArrays(1, &skyboxVAO);
	glGenBuffers(1, &skyboxVBO);
	glBindVertexArray(skyboxVAO);
	glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	// load textures
	// -------------
	//unsigned int cubeTexture = loadTexture(FileSystem::getPath("resources/textures/marble.jpg").c_str());
	unsigned int albedo = loadTexture("E:/ex3/BRDF/su.jpg");    //加载纹理
	vector<std::string> faces
	{
		"E:/ex3/BRDF/skybox/right.tga",
		"E:/ex3/BRDF/skybox/left.tga",
		"E:/ex3/BRDF/skybox/top.tga",
		"E:/ex3/BRDF/skybox/bottom.tga",
		"E:/ex3/BRDF/skybox/front.tga",
		"E:/ex3/BRDF/skybox/back.tga"
	};
	unsigned int cubemapTexture = loadCubemap(faces);

	// shader configuration
	// --------------------
	skyboxShader.use();
	skyboxShader.setInt("skybox", 0);


	// render loop
	// -----------
	while (!glfwWindowShouldClose(window))
	{
		// per-frame time logic
		// --------------------
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		// input
		// -----
		processInput(window);

		// render
		// ------
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


		lightingShader.use();

		// 设置光源，光照，摄像机位置
		lightingShader.setVec3("lightColors", 700.0f, 300.0f, 200.0f);
		lightingShader.setVec3("lightPositions", lightPos);
		lightingShader.setVec3("camPos", camera.Position);

		//绑定纹理,在绑定纹理之前先激活纹理单元
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, albedo);
	/*	glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, normal);
		glActiveTexture(GL_TEXTURE2);
		glBindTexture(GL_TEXTURE_2D, metallic);
		glActiveTexture(GL_TEXTURE3);
		glBindTexture(GL_TEXTURE_2D, roughness);
		glActiveTexture(GL_TEXTURE4);
		glBindTexture(GL_TEXTURE_2D, ao);*/


		// view/projection transformations 投影矩阵与视角变换
		glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
		glm::mat4 view = camera.GetViewMatrix();
		lightingShader.setMat4("projection", projection);
		lightingShader.setMat4("view", view);
		//对机器人进行绘制
		glBindVertexArray(cubeVAO);
		//画身体
		glm::mat4 model = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
		model = glm::translate(model, glm::vec3(0.0f, 0.0f, walk));
		model = glm::scale(model, glm::vec3(3.0f, 4.0f, 1.0f));
		//传入变换矩阵
		lightingShader.setMat4("model", model);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		//画脖子
		model = glm::mat4(1.0f);
		model = glm::translate(model, glm::vec3(0.0f, 0.45f, walk));
		model = glm::scale(model, glm::vec3(1.0f, 0.5f, 1.0f));
		lightingShader.setMat4("model", model);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		//画头
		model = glm::mat4(1.0f);
		model = glm::translate(model, glm::vec3(0.0f, 0.7f, walk));
		model = glm::scale(model, glm::vec3(2.0f, 2.0f, 2.0f));
		lightingShader.setMat4("model", model);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		//画左手臂
		model = glm::mat4(1.0f);
		model = glm::translate(model, glm::vec3(0.375f, 0.2f, walk));
		if (op == 1)
		{
			model = glm::translate(model, glm::vec3(0.0f, 0.2f, 0.0f));   //平移回去
			model = glm::rotate(model, (float)angel1, glm::vec3(0.0f, 0.0f, 1.0f));  //绕Z轴旋转
			model = glm::translate(model, glm::vec3(0.0f, -0.2f, 0.0f));   //为了旋转先向下平移
		}
		if (op == 2)
		{
			model = glm::translate(model, glm::vec3(0.0f, 0.2f, 0.0f));   //平移回去
			model = glm::rotate(model, (float)larm1, glm::vec3(-1.0f, 0.0f, 0.0f));  //绕x轴旋转
			model = glm::translate(model, glm::vec3(0.0f, -0.2f, 0.0f));   //为了旋转先向下平移
		}
		model = glm::scale(model, glm::vec3(0.75f, 2.0f, 1.0f));     //放缩
		lightingShader.setMat4("model", model);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		//画左臂关节
		model = glm::mat4(1.0f);
		model = glm::translate(model, glm::vec3(0.375f, -0.05f, walk));
		if (op == 1) //动胳膊的操作
		{
			model = glm::translate(model, glm::vec3(0.0f, 0.45f, 0.0f));   //平移回去
			model = glm::rotate(model, (float)angel1, glm::vec3(0.0f, 0.0f, 1.0f));  //绕Z轴旋转
			model = glm::translate(model, glm::vec3(0.0f, -0.45f, 0.0f));   //为了大臂旋转先向下平移
		}
		if (op == 2) //行走
		{
			model = glm::translate(model, glm::vec3(0.0f, 0.45f, 0.0f));   //平移回去
			model = glm::rotate(model, (float)larm1, glm::vec3(-1.0f, 0.0f, 0.0f));  //绕x轴旋转
			model = glm::translate(model, glm::vec3(0.0f, -0.45f, 0.0f));   //为了大臂旋转先向下平移
		}
		model = glm::scale(model, glm::vec3(0.65f, 0.5f, 0.707f));
		model = glm::rotate(model, glm::radians((float)45.0), glm::vec3(1.0f, 0.0f, 0.0f));
		lightingShader.setMat4("model", model);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		//画左手肘
		model = glm::mat4(1.0f);
		model = glm::translate(model, glm::vec3(0.375f, -0.3f, walk));
		if (op == 1)
		{
			model = glm::translate(model, glm::vec3(0.0f, 0.7f, 0.0f));   //平移回去
			model = glm::rotate(model, (float)angel1, glm::vec3(0.0f, 0.0f, 1.0f));  //绕Z轴旋转，转大臂
			model = glm::translate(model, glm::vec3(0.0f, -0.45f, 0.0f));   //为了大臂旋转再往下平移
			model = glm::rotate(model, (float)angel2, glm::vec3(0.0f, 0.0f, 1.0f));  //绕Z轴旋转，转自己
			model = glm::translate(model, glm::vec3(0.0f, -0.25f, 0.0f));   //为了旋转先向下平移
		}
		if (op == 2)
		{
			model = glm::translate(model, glm::vec3(0.0f, 0.7f, 0.0f));   //平移回去
			model = glm::rotate(model, (float)larm1, glm::vec3(-1.0f, 0.0f, 0.0f));  //绕x轴旋转，转大臂
			model = glm::translate(model, glm::vec3(0.0f, -0.45f, 0.0f));   //为了大臂旋转再往下平移
			model = glm::rotate(model, (float)larm2, glm::vec3(-1.0f, 0.0f, 0.0f));  //绕Z轴旋转，转自己
			model = glm::translate(model, glm::vec3(0.0f, -0.25f, 0.0f));   //为了旋转先向下平移
		}
		model = glm::scale(model, glm::vec3(0.75f, 2.0f, 1.0f));
		lightingShader.setMat4("model", model);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		//画右手臂
		model = glm::mat4(1.0f);
		model = glm::translate(model, glm::vec3(-0.375f, 0.2f, walk));
		if (op == 2) //正行走
		{
			model = glm::translate(model, glm::vec3(0.0f, 0.2f, 0.0f));   //平移回去
			model = glm::rotate(model, (float)rarm1, glm::vec3(-1.0f, 0.0f, 0.0f));  //绕x轴旋转
			model = glm::translate(model, glm::vec3(0.0f, -0.2f, 0.0f));   //为了旋转先向下平移
		}
		model = glm::scale(model, glm::vec3(0.75f, 2.0f, 1.0f));
		lightingShader.setMat4("model", model);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		//画右臂关节
		model = glm::mat4(1.0f);
		model = glm::translate(model, glm::vec3(-0.375f, -0.05f, walk));
		if (op == 2) //行走
		{
			model = glm::translate(model, glm::vec3(0.0f, 0.45f, 0.0f));   //平移回去
			model = glm::rotate(model, (float)rarm1, glm::vec3(-1.0f, 0.0f, 0.0f));  //绕x轴旋转
			model = glm::translate(model, glm::vec3(0.0f, -0.45f, 0.0f));   //为了大臂旋转先向下平移
		}
		model = glm::scale(model, glm::vec3(0.65f, 1.0f, 0.707f));
		model = glm::rotate(model, glm::radians((float)45.0), glm::vec3(1.0f, 0.0f, 0.0f));
		lightingShader.setMat4("model", model);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		//画右手肘
		model = glm::mat4(1.0f);
		model = glm::translate(model, glm::vec3(-0.375f, -0.3f, walk));
		if (op == 2)  //现在是行走操作
		{
			model = glm::translate(model, glm::vec3(0.0f, 0.7f, 0.0f));   //平移回去
			model = glm::rotate(model, (float)rarm1, glm::vec3(-1.0f, 0.0f, 0.0f));  //绕x轴旋转，转大臂
			model = glm::translate(model, glm::vec3(0.0f, -0.45f, 0.0f));   //为了大臂旋转再往下平移
			model = glm::rotate(model, (float)rarm2, glm::vec3(-1.0f, 0.0f, 0.0f));  //绕x轴旋转，转自己
			model = glm::translate(model, glm::vec3(0.0f, -0.25f, 0.0f));   //为了旋转先向下平移
		}
		model = glm::scale(model, glm::vec3(0.75f, 2.0f, 1.0f));
		//model = glm::rotate(model, glm::radians((float)45.0), glm::vec3(1.0f, 0.0f, 0.0f));
		lightingShader.setMat4("model", model);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		//画左大腿
		model = glm::mat4(1.0f);
		model = glm::translate(model, glm::vec3(0.15f, -0.6f, walk));
		if (op == 2)
		{
			model = glm::translate(model, glm::vec3(0.0f, 0.2f, 0.0f));   //平移回去
			model = glm::rotate(model, (float)lleg1, glm::vec3(-1.0, 0.0f, 0.0f));  //绕x轴旋转
			model = glm::translate(model, glm::vec3(0.0f, -0.2f, 0.0f));   //为了旋转先向下平移
		}
		model = glm::scale(model, glm::vec3(1.0f, 2.0f, 1.0f));
		lightingShader.setMat4("model", model);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		//画左腿关节
		model = glm::mat4(1.0f);
		model = glm::translate(model, glm::vec3(0.15f, -0.85f, walk));
		if (op == 2) //行走
		{
			model = glm::translate(model, glm::vec3(0.0f, 0.45f, 0.0f));   //平移回去
			model = glm::rotate(model, (float)lleg1, glm::vec3(-1.0f, 0.0f, 0.0f));  //绕x轴旋转
			model = glm::translate(model, glm::vec3(0.0f, -0.45f, 0.0f));   //为了大腿旋转先向下平移
		}
		model = glm::scale(model, glm::vec3(0.75f, 0.5f, 0.707f));
		model = glm::rotate(model, glm::radians((float)45.0), glm::vec3(1.0f, 0.0f, 0.0f));
		lightingShader.setMat4("model", model);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		//画左小腿
		model = glm::mat4(1.0f);
		model = glm::translate(model, glm::vec3(0.15f, -1.1f, walk));
		if (op == 2)
		{
			model = glm::translate(model, glm::vec3(0.0f, 0.7f, 0.0f));   //平移回去
			model = glm::rotate(model, (float)lleg1, glm::vec3(-1.0f, 0.0f, 0.0f));  //绕x轴旋转，转大臂
			model = glm::translate(model, glm::vec3(0.0f, -0.45f, 0.0f));   //为了大臂旋转再往下平移
			model = glm::rotate(model, (float)lleg2, glm::vec3(1.0f, 0.0f, 0.0f));  //绕x轴旋转，转自己
			model = glm::translate(model, glm::vec3(0.0f, -0.25f, 0.0f));   //为了旋转先向下平移
		}
		model = glm::scale(model, glm::vec3(0.5f, 2.0f, 0.5f));
		lightingShader.setMat4("model", model);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		//画右大腿
		model = glm::mat4(1.0f);
		model = glm::translate(model, glm::vec3(-0.15f, -0.6f, walk));
		if (op == 2)
		{
			model = glm::translate(model, glm::vec3(0.0f, 0.2f, 0.0f));   //平移回去
			model = glm::rotate(model, (float)rleg1, glm::vec3(-1.0, 0.0f, 0.0f));  //绕x轴旋转
			model = glm::translate(model, glm::vec3(0.0f, -0.2f, 0.0f));   //为了旋转先向下平移
		}
		model = glm::scale(model, glm::vec3(1.0f, 2.0f, 1.0f));
		lightingShader.setMat4("model", model);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		//画右腿关节
		model = glm::mat4(1.0f);
		model = glm::translate(model, glm::vec3(-0.15f, -0.85f, walk));
		if (op == 2) //行走
		{
			model = glm::translate(model, glm::vec3(0.0f, 0.45f, 0.0f));   //平移回去
			model = glm::rotate(model, (float)rleg1, glm::vec3(-1.0f, 0.0f, 0.0f));  //绕x轴旋转
			model = glm::translate(model, glm::vec3(0.0f, -0.45f, 0.0f));   //为了大腿旋转先向下平移
		}
		model = glm::scale(model, glm::vec3(0.75f, 0.5f, 0.707f));
		model = glm::rotate(model, glm::radians((float)45.0), glm::vec3(1.0f, 0.0f, 0.0f));
		lightingShader.setMat4("model", model);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		//画右小腿
		model = glm::mat4(1.0f);
		model = glm::translate(model, glm::vec3(-0.15f, -1.1f, walk));
		if (op == 2)
		{
			model = glm::translate(model, glm::vec3(0.0f, 0.7f, 0.0f));   //平移回去
			model = glm::rotate(model, (float)rleg1, glm::vec3(-1.0f, 0.0f, 0.0f));  //绕x轴旋转，转大臂
			model = glm::translate(model, glm::vec3(0.0f, -0.45f, 0.0f));   //为了大臂旋转再往下平移
			model = glm::rotate(model, (float)rleg2, glm::vec3(1.0f, 0.0f, 0.0f));  //绕x轴旋转，转自己
			model = glm::translate(model, glm::vec3(0.0f, -0.25f, 0.0f));   //为了旋转先向下平移
		}
		model = glm::scale(model, glm::vec3(0.5f, 2.0f, 0.5f));
		lightingShader.setMat4("model", model);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		//画左脚
		model = glm::mat4(1.0f);
		model = glm::translate(model, glm::vec3(0.15f, -1.35f, 0.1f+ walk));
		if (op == 2)
		{
			model = glm::translate(model, glm::vec3(0.0f, 0.95f, 0.0f));   //平移回去
			model = glm::rotate(model, (float)lleg1, glm::vec3(-1.0f, 0.0f, 0.0f));  //绕x轴旋转，转大臂
			model = glm::translate(model, glm::vec3(0.0f, -0.45f, 0.0f));   //为了大臂旋转再往下平移
			model = glm::rotate(model, (float)lleg2, glm::vec3(1.0f, 0.0f, 0.0f));  //绕x轴旋转，转自己
			model = glm::translate(model, glm::vec3(0.0f, -0.5f, 0.0f));   //为了旋转先向下平移
		}
		model = glm::scale(model, glm::vec3(0.5f, 0.5f, 1.5f));
		lightingShader.setMat4("model", model);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		//画右脚
		model = glm::mat4(1.0f);
		model = glm::translate(model, glm::vec3(-0.15f, -1.35f, 0.1f+ walk));
		if (op == 2)
		{
			model = glm::translate(model, glm::vec3(0.0f, 0.95f, 0.0f));   //平移回去
			model = glm::rotate(model, (float)rleg1, glm::vec3(-1.0f, 0.0f, 0.0f));  //绕x轴旋转，转大臂
			model = glm::translate(model, glm::vec3(0.0f, -0.45f, 0.0f));   //为了大臂旋转再往下平移
			model = glm::rotate(model, (float)rleg2, glm::vec3(1.0f, 0.0f, 0.0f));  //绕x轴旋转，转自己
			model = glm::translate(model, glm::vec3(0.0f, -0.5f, 0.0f));   //为了旋转先向下平移
		}
		model = glm::scale(model, glm::vec3(0.5f, 0.5f, 1.5f));
		lightingShader.setMat4("model", model);
		glDrawArrays(GL_TRIANGLES, 0, 36);


		// also draw the lamp object
		lampShader.use();
		lampShader.setMat4("projection", projection);   //观察矩阵
		lampShader.setMat4("view", view);             //透视投影矩阵
		model = glm::mat4(1.0f);
		model = glm::translate(model, lightPos);
		model = glm::scale(model, glm::vec3(0.2f)); // a smaller cube
		lampShader.setMat4("model", model);

		glBindVertexArray(lightVAO);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		
		// draw skybox as last
		glDepthFunc(GL_LEQUAL);  // change depth function so depth test passes when values are equal to depth buffer's content
		skyboxShader.use();
		view = glm::mat4(glm::mat3(camera.GetViewMatrix())); // remove translation from the view matrix
		skyboxShader.setMat4("view", view);
		skyboxShader.setMat4("projection", projection);
		// skybox cube
		glBindVertexArray(skyboxVAO);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		glBindVertexArray(0);
		glDepthFunc(GL_LESS); // set depth function back to default

		// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
		// -------------------------------------------------------------------------------
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	// optional: de-allocate all resources once they've outlived their purpose:
	// ------------------------------------------------------------------------
	glDeleteVertexArrays(1, &cubeVAO);
	glDeleteVertexArrays(1, &lightVAO);
	glDeleteVertexArrays(1, &skyboxVAO);
	glDeleteBuffers(1, &VBO);
	glDeleteBuffers(1, &skyboxVBO);

	// glfw: terminate, clearing all previously allocated GLFW resources.
	// ------------------------------------------------------------------
	glfwTerminate();
	return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		camera.ProcessKeyboard(FORWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		camera.ProcessKeyboard(BACKWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		camera.ProcessKeyboard(LEFT, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		camera.ProcessKeyboard(RIGHT, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS)
	{
		op = 1;
		px = px + 0.001;
		bool can = IK(px, py, 0.45, 0.45, angel1, angel2);
		if (!can)
			px = px - 0.001;
		cout << px << " " << py << " " << angel1 << " " << angel2;
	}
	if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS)
	{
		op = 1;
		py = py + 0.001;
		bool can = IK(px, py, 0.45, 0.45, angel1, angel2);
		if (!can)
			py = py - 0.001;
		cout << px << " " << py << " " << angel1 << " " << angel2;
	}
	if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS)
	{
		op = 1;
		px = px - 0.001;
		bool can = IK(px, py, 0.45, 0.45, angel1, angel2);
		if (!can)
			px = px + 0.001;
		cout << px << " " << py << " " << angel1 << " " << angel2;
	}
	if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS)
	{
		op = 1;
		py = py - 0.001;
		bool can = IK(px, py, 0.45, 0.45, angel1, angel2);
		if (!can)
			py = py + 0.001;
		cout << px << " " << py << " " << angel1 << " " << angel2;
	}

	if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
	{
		op = 2;
		//右脚抬起左手前进 右手后退
		if (ss == 1)
		{
			rly += 0.005;
			IK3(rlx, rly, 0.45, 0.45, rleg1, rleg2);
			larm1 += glm::radians((float)0.45);
			larm2 += glm::radians((float)0.45);
			rarm1 -= glm::radians((float)0.45);
			rarm2 -= glm::radians((float)0.45);
			cout << "haha1" << endl;
			cout << rly << endl;
			if (rly >= -0.4)
			{
				ss = 2;
				cout << ss << endl;
			}
		}
		else if (ss == 2)
		{//右脚放下 手臂收回
			//cout << rly << " " << rleg1 << " " << rleg2 << endl;
			cout << "haha2" << endl;
			rly -= 0.005;
			IK3(rlx, rly, 0.45, 0.45, rleg1, rleg2);
			larm1 -= glm::radians((float)0.45);
			larm2 -= glm::radians((float)0.45);
			rarm1 += glm::radians((float)0.45);
			rarm2 += glm::radians((float)0.45);
			if (rly < -0.89)
				ss = 3;
			cout << larm1 << " " << larm2 << endl;
		}
		else if (ss == 3)
		{//左脚抬起，右臂前 ，左臂后
			cout << "haha3" << endl;
			lly += 0.005;
			IK3(llx, lly, 0.45, 0.45, lleg1, lleg2);
			larm1 -= glm::radians((float)0.45);
			larm2 -= glm::radians((float)0.45);
			rarm1 += glm::radians((float)0.45);
			rarm2 += glm::radians((float)0.45);

			if (lly >= -0.4)
				ss = 4;
		}
		else if (ss == 4)
		{//手臂收回
			lly -= 0.005;
			IK3(llx, lly, 0.45, 0.45, lleg1, lleg2);
			larm1 += glm::radians((float)0.45);
			larm2 += glm::radians((float)0.45);
			rarm1 -= glm::radians((float)0.45);
			rarm2 -= glm::radians((float)0.45);
			if (lly < -0.89)
				ss = 1;
		}
		if (direction == 1)
			walk += 0.005;
		else
			walk -= 0.005;
	}
	if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS)
	{
		direction = 1;
	}
	if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS)
	{
		direction = 0;
	}
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}


// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

	lastX = xpos;
	lastY = ypos;

	camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	camera.ProcessMouseScroll(yoffset);
}

// utility function for loading a 2D texture from file
// ---------------------------------------------------
unsigned int loadTexture(char const * path)
{ //生成纹理
	unsigned int textureID;
	glGenTextures(1, &textureID); //生成纹理的数量
	int width, height, nrComponents;
	unsigned char *data = stbi_load(path, &width, &height, &nrComponents, 0);
	if (data)
	{
		GLenum format;
		if (nrComponents == 1)
			format = GL_RED;
		else if (nrComponents == 3)
			format = GL_RGB;
		else if (nrComponents == 4)
			format = GL_RGBA;
		//绑定它，让之后任何的纹理指令都可以配置当前绑定的纹理
		glBindTexture(GL_TEXTURE_2D, textureID);
		//使用前面载入的图片数据生成一个纹理
		glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
		glGenerateMipmap(GL_TEXTURE_2D);

		//设置纹理参数
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		stbi_image_free(data);
	}
	else
	{
		std::cout << "Texture failed to load at path: " << path << std::endl;
		stbi_image_free(data);
	}

	return textureID;
}

unsigned int loadCubemap(vector<std::string> faces)
{
	unsigned int textureID;
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

	int width, height, nrChannels;
	for (unsigned int i = 0; i < faces.size(); i++)
	{
		unsigned char *data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
		if (data)
		{
			glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
			stbi_image_free(data);
		}
		else
		{
			std::cout << "Cubemap texture failed to load at path: " << faces[i] << std::endl;
			stbi_image_free(data);
		}
	}
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	return textureID;
}

bool IK(double x, double y, double l1, double l2, double &theta1, double &theta2)
{
	if (x*x + y * y > (l1 + l2)*(l1 + l2))
		return false;
	double a;
	double PI = 3.1415926535;
	a = (x*x + y * y - l1 * l1 - l2 * l2) / (2 * l1*l2);
	theta2 = acos(a);
	if (y < 0)
	{
		//theta1 = (y * l2*sin(theta2) + x * (l1 + l2 * cos(theta2))) / (x*l2*sin(theta2) - y * (l1 + l2 * cos(theta2)));
		theta1 = atan(x / abs(y)) - atan((l2*sin(theta2)) / (l1 + l2 * cos(theta2)));
	}
	else
	{

		a = (x * x + y * y + l1 * l1 - l2 * l2) / (2 * l1*sqrt(x * x + y * y));
		theta1 = -acos(a) + PI / 2 + atan(y / x);
	}
	return true;
}

bool IK1(double x, double y, double l1, double l2, double &theta1, double &theta2)
{
	cout << x << " " << y << " " << theta1 << " " << theta2 << endl;
	if (x*x + y * y > (l1 + l2)*(l1 + l2))
		return false;
	double a;
	double PI = 3.1415926535;
	a = (x*x + y * y - l1 * l1 - l2 * l2) / (2 * l1*l2);
	theta2 = acos(a);
	if (x < 0)
	{
		//theta1 = (y * l2*sin(theta2) + x * (l1 + l2 * cos(theta2))) / (x*l2*sin(theta2) - y * (l1 + l2 * cos(theta2)));
		theta1 = atan(x / abs(y)) - atan((l2*sin(theta2)) / (l1 + l2 * cos(theta2)));
	}
	else
	{

		a = (x * x + y * y + l1 * l1 - l2 * l2) / (2 * l1*sqrt(x * x + y * y));
		theta1 = -(acos(a) + atan(x / y));
		cout << theta1 << endl;
	}
	cout << x << " " << y << " " << theta1 << " " << theta2 << endl;
	return true;
}

bool IK3(double x, double y, double l1, double l2, double &theta1, double &theta2)
{
	if (x*x + y * y > (l1 + l2)*(l1 + l2))
		 return false;
	double a;
	double PI = 3.1415926535;
	a = (x*x + y * y - l1 * l1 - l2 * l2) / (2 * l1*l2);
	theta2 = acos(a);
	a = (x * x + y * y + l1 * l1 - l2 * l2) / (2 * l1*sqrt(x * x + y * y));
	theta1 = acos(a) + atan(x / abs(y));
	//cout << x << " " << y << " " << theta1 << " " << theta2 << endl;
	return true;
}