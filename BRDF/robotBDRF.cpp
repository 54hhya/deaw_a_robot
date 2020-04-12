#include <glad/glad.h>
#include <GLFW/glfw3.h>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "shader.h"
#include "camera.h"

#include <iostream>
using namespace std;

struct theta {
	double theta1 = 0 ;
	double theta2 = 0;
};

double PI = 3.1415926535;
double xx = 0;  //手臂IK
double yy = 0;
double walkyy = 0; //走路脚步IK
unsigned int texture1;
unsigned int textureID;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void cubeBind(const char * path); //绑定VBO 
void processInput(GLFWwindow *window);
unsigned int loadCubemap(vector<std::string> faces); //天空盒纹理加载
theta IK(double x, double y,double L1, double L2);  //IK逆向动力学
GLFWwindow* window;

glm::vec3 pos = glm::vec3(0.0, 0.0, -3.0f);  //机器人位置
glm::vec3 label = glm::vec3(0.0, 1.0, 0.0);  //大轴位置
float Langle = 0;
glm::vec3 rotation = glm::vec3(1.0, 0.0, 0.0);  //转动轴
glm::vec3 IKRotation = glm::vec3(0.0, 0.0, 1.0);  //IK轴
 //关节转动角度
float angle[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//IK关节转动角度
float IKAngle[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//转动标志
int flag[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
int IKFlag = 1;  //行走手臂摆动方向
//cube点
float vertices[] = {
	-0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f, 0.0f,
	 0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f, 0.0f,
	 0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f, 1.0f,
	 0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f, 1.0f,
	-0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f, 1.0f,
	-0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f, 0.0f,

	-0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f, 0.0f,
	 0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f, 0.0f,
	 0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f, 1.0f,
	 0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f, 1.0f,
	-0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f, 1.0f,
	-0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f, 0.0f,

	-0.5f,  0.5f,  0.5f,  -1.0f,  0.0f,  0.0f,  1.0f, 0.0f,
	-0.5f,  0.5f, -0.5f,  -1.0f,  0.0f,  0.0f,  1.0f, 1.0f,
	-0.5f, -0.5f, -0.5f,  -1.0f,  0.0f,  0.0f,  0.0f, 1.0f,
	-0.5f, -0.5f, -0.5f,  -1.0f,  0.0f,  0.0f,  0.0f, 1.0f,
	-0.5f, -0.5f,  0.5f,  -1.0f,  0.0f,  0.0f,  0.0f, 0.0f,
	-0.5f,  0.5f,  0.5f,  -1.0f,  0.0f,  0.0f,  1.0f, 0.0f,

	 0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f,
	 0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f,
	 0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f, 1.0f,
	 0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f, 1.0f,
	 0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  0.0f, 0.0f,
	 0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f,

	-0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f, 1.0f,
	 0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  1.0f, 1.0f,
	 0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f, 0.0f,
	 0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f, 0.0f,
	-0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  0.0f, 0.0f,
	-0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f, 1.0f,

	-0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f,
	 0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f, 1.0f, 1.0f,
	 0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f,
	 0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f,
	-0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f, 0.0f, 0.0f,
	-0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f
};
//天空盒点
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


//天空盒图片
vector<std::string> faces
{
	/*"F:/code2/city/posx.jpg",
	"F:/code2/city/negx.jpg",
	"F:/code2/city/posy.jpg",
	"F:/code2/city/negy.jpg",
	"F:/code2/city/posz.jpg",
	"F:/code2/city/negz.jpg"*/
	"E:/ex3/BRDF/skybox/right.png",
		"E:/ex3/BRDF/skybox/left.png",
		"E:/ex3/BRDF/skybox/top.tga",
		"E:/ex3/BRDF/skybox/bottom.tga",
		"E:/ex3/BRDF/skybox/front.tga",
		"E:/ex3/BRDF/skybox/back.png"

};


unsigned int VBO, VAO;  //缓冲区
unsigned int skyboxVAO, skyboxVBO;

//窗口设置
const unsigned int SCR_WIDTH = 1000;
const unsigned int SCR_HEIGHT = 1000;

//摄像机设置
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

//时间设置
float deltaTime = 0.0f;
float lastFrame = 0.0f;

//cube的位置
glm::vec3 cubePositions[] = {
	glm::vec3(0.0f,  1.875f,  -5.0f),   //头
	glm::vec3(0.0f,  0.5f,  -5.0f),   //身体
	glm::vec3(-0.85f, 1.0f,  -5.0f),   //左大臂
	glm::vec3(-0.85f, 0.0f,  -5.0f),   //左小臂
	glm::vec3(0.85f,  1.0f,  -5.0f),   //右大臂
	glm::vec3(0.85f,  0.0f,  -5.0f),   //右小臂
	glm::vec3(-0.3f, -1.25f,  -5.0f),   //左大腿
	glm::vec3(-0.3f, -2.75f,  -5.0f),   //左小腿
	glm::vec3(0.3f,  -1.25f,  -5.0f),   //右大腿
	glm::vec3(0.3f,  -2.75f,  -5.0f),   //右小腿
	glm::vec3(-0.625f,  1.375f,  -5.0f),   //左大臂接点
	glm::vec3(0.625f,  1.375f,  -5.0f),   //右大臂接点
};

//cube的长宽高
glm::vec3 cubeLWH[] = {
	glm::vec3(0.5f,  0.75f,  0.5f),
	glm::vec3(1.0f,  2.0f,  1.0f),
	glm::vec3(0.2f,  1.0f,  0.2f),
	glm::vec3(0.2f,  1.0f,  0.2f),
	glm::vec3(0.2f,  1.0f,  0.2f),
	glm::vec3(0.2f,  1.0f,  0.2f),
	glm::vec3(0.2f,  1.5f,  0.2f),
	glm::vec3(0.2f,  1.5f,  0.2f),
	glm::vec3(0.2f,  1.5f,  0.2f),
	glm::vec3(0.2f,  1.5f,  0.2f),
	glm::vec3(0.25f,  0.25f,  0.25f),
	glm::vec3(0.25f,  0.25f,  0.25f)
};

//关节点位置
glm::vec3 joint[] = {
	glm::vec3(0.0f,   1.5f,  -5.0f),  //脑袋和身体
	glm::vec3(0.0f,   1.5f,  -5.0f),  //无
	glm::vec3(-0.75f, 1.5f,  -5.0f),  //身体和左胳膊
	glm::vec3(-0.85f,  0.5f,  -5.0f),  //左大臂和左小臂
	glm::vec3(0.75f,  1.5f,  -5.0f),  //身体和右胳膊
	glm::vec3(0.85f,   0.5f,  -5.0f),  //右大臂和右小臂
	glm::vec3(-0.3f, -0.5f,  -5.0f),  //身体和左大腿
	glm::vec3(-0.3f, -2.0f,  -5.0f),  //左大腿和左小腿
	glm::vec3(0.3f,  -0.5f,  -5.0f),  //身体和右大腿
	glm::vec3(0.3f,  -2.0f,  -5.0f),   //右大腿和右小腿
	glm::vec3(-0.3f,  1.5f,  -5.0f),  //左大臂接点
	glm::vec3(0.3f,  1.5f,  -5.0f)   //右大臂接点
};

//灯光位置
glm::vec3 lightPositions[] = {
	//glm::vec3(-10.0f,  15.0f, -5.0f),
	glm::vec3(5.0f,  15.0f, 0.0f),
	glm::vec3(5.0f,  15.0f, 0.0f),
	glm::vec3(5.0f,  15.0f, 0.0f),
	glm::vec3(5.0f,  15.0f, 0.0f),
};

//灯光颜色
glm::vec3 lightColors[] = {
	glm::vec3(1000.0f, 1000.0f, 1000.0f),
	glm::vec3(1000.0f, 1000.0f, 1000.0f),
	glm::vec3(1000.0f, 1000.0f, 1000.0f),
	glm::vec3(1000.0f, 1000.0f, 1000.0f)
};


void init()
{
	// glfw窗口设置
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// glfw窗口创建
	window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
	if (window == NULL)
	{
		cout << "glfw窗口创建失败" << endl;
		glfwTerminate();
		return;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);

	//鼠标设置
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	//初始化GLAD
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		cout << "GLAD初始化失败" << endl;
		return;
	}

	//深度测试开启
	glEnable(GL_DEPTH_TEST);
}

int main()
{
	// 初始化
	init();

	//VAO 纹理设置绑定
	//cubeBind("F:/code1/wood.png");

	//创建着色器，绑定纹理
	Shader ourShader("E:/ex3/BRDF/shader2.vs", "E:/ex3/BRDF/shader2.fs");
	cubeBind("E:/ex3/BRDF/su.jpg");
	ourShader.use();
	ourShader.setInt("albedoMap", 1);
	ourShader.setFloat("ao", 1.0f);
	ourShader.setFloat("metallic", 0.85);
	ourShader.setFloat("roughness", 0.40);//glm::clamp(0.3f, 0.05f, 1.0f));*/

	Shader skyboxShader("E:/ex3/BRDF/skybox.vs.txt", "E:/ex3/BRDF/skybox.fs.txt");

	// skybox VAO
	glGenVertexArrays(1, &skyboxVAO);
	glGenBuffers(1, &skyboxVBO);
	glBindVertexArray(skyboxVAO);
	glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	unsigned int cubemapTexture = loadCubemap(faces);
	skyboxShader.use();
	skyboxShader.setInt("skybox", 0);

	//渲染
	while (!glfwWindowShouldClose(window))
	{
		//时间设置
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		// 输入
		processInput(window);
		//背景颜色
		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//cubeBind("F:/code1/wood.png");
		//projection
		ourShader.use();
		glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
		ourShader.setMat4("projection", projection);

		// camera
		glm::mat4 view = camera.GetViewMatrix();
		ourShader.setVec3("camPos", camera.Position);
		ourShader.setMat4("view", view);
		for (unsigned int i = 0; i < sizeof(lightPositions) / sizeof(lightPositions[0]); ++i)
		{
			glm::vec3 newPos = lightPositions[i];//+ glm::vec3(sin(glfwGetTime() * 5.0) * 5.0, 0.0, 0.0);
			newPos = lightPositions[i];
			ourShader.setVec3("lightPositions[" + std::to_string(i) + "]", newPos);
			ourShader.setVec3("lightColors[" + std::to_string(i) + "]", lightColors[i]);
		}
		//箱子绑定绘制
		for (unsigned int i = 0; i < 12; i++)
		{
			glBindVertexArray(VAO);
			glm::mat4 model = glm::mat4(1.0f); //model位置 总体model位置
			model = glm::translate(model, pos);               //平移
			model = glm::translate(model, glm::vec3(0.0, 0.0, joint[0].z));
			model = glm::rotate(model, glm::radians(Langle), label);               //整体旋转
			model = glm::translate(model, glm::vec3(0.0, 0.0, -joint[0].z));

			if (i == 3 || i == 5 || i == 7 || i == 9) { //绕大轴转动
				model = glm::translate(model, joint[i - 1]);   //旋转点
				model = glm::rotate(model, glm::radians(angle[i - 1]), rotation);  //转动
				model = glm::rotate(model, glm::radians(IKAngle[i - 1]), IKRotation);  //走路转动
				model = glm::translate(model, -joint[i - 1]);   //旋转点
			}
			model = glm::translate(model, joint[i]);   //旋转点
			//设置转轴
			if (i == 2 || i == 3 || i == 4 || i == 5 || i == 6 || i == 7 || i == 8 || i == 9) {//绕小关节点转动
				//angle[i] += (float)glfwGetTime() ;
				model = glm::rotate(model, glm::radians(angle[i]), rotation);  //转动
				model = glm::rotate(model, glm::radians(IKAngle[i]), IKRotation);  //转动
			}
			model = glm::translate(model, cubePositions[i] - joint[i]);  //平移
			model = glm::scale(model, cubeLWH[i]);             //放缩


			ourShader.setMat4("model", model);

			glBindVertexArray(VAO);
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, texture1);
			glDrawArrays(GL_TRIANGLES, 0, 36);
			glBindVertexArray(0);
		}
			//加载天空盒
			glDepthFunc(GL_LEQUAL);  //改变深度函数
			skyboxShader.use();
			view = glm::mat4(glm::mat3(camera.GetViewMatrix())); //重新设置view
			skyboxShader.setMat4("view", view);
			skyboxShader.setMat4("projection", projection);

			//天空盒VAO绑定
			glBindVertexArray(skyboxVAO);
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);
			glDrawArrays(GL_TRIANGLES, 0, 36);
			glBindVertexArray(0);
			glDepthFunc(GL_LESS); //将深度函数设置回默认值

			glfwSwapBuffers(window);
			glfwPollEvents();
	}
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);
	glDeleteVertexArrays(1, &skyboxVAO);
	glDeleteBuffers(1, &skyboxVBO);

	glfwTerminate();
	return 0;
}

//逆向运动学
theta IK(double x, double y, double L1, double L2) {
	theta beta;
	double beta2Cos = (x * x + y * y - L1 * L1 - L2 * L2)/( 2 * L1 * L2 );
	double binusCos = (x * x + y * y + L1 * L1 - L2 * L2) / (2 * L1 * sqrt(x * x + y * y));
	if (abs(beta2Cos) > 1 || abs(binusCos) > 1)
	{
		cout << "IK无法达到" << endl;
		return beta;
	}
	beta.theta2 =  (acos(beta2Cos)) / PI * 180;
	if (y > 0) 
		beta.theta1 = (atan(abs(y / x)) + PI / 2 - acos(binusCos))/ PI * 180;
	else 
		beta.theta1 = (atan(abs(x / y)) - acos(binusCos)) / PI * 180;
	//cout << "xx" << x << " " << "yy" << y <<" "<< "1" << beta.theta1<<"  acos(beta2Cos)"<< acos(beta2Cos) <<" 2"<<beta.theta2<<endl;

	return beta;
}



//机器人行走 -1向前 1向后
void walk(int direction) {
	//向前 向后
	float speed = 0.05f;
	pos.x += direction * speed * abs(sin(Langle * PI / 180));
	pos.z += direction * speed * abs(cos(Langle * PI / 180));

	float Wspeed = 0.5f;
	//摆动手臂
	angle[2] += Wspeed * IKFlag;
	angle[3] = 0.5 * angle[2];
	angle[4] -= Wspeed * IKFlag;
	angle[5] = 0.5 * angle[4];
	if (abs(angle[2]) >= 40)
		IKFlag = -IKFlag;

	int footFlag = 0;
	if (angle[2] >= 0 && IKFlag > 0)
		footFlag = 3;
	else if (angle[2] > 0 && IKFlag < 0)
		footFlag = 4;
	else if (angle[2] <= 0 && IKFlag < 0)
		footFlag = 1;
	else
		footFlag = 2;
	//IK摆动脚
	float upFoot = 0.02f;
	if (footFlag % 2 == 1)
		walkyy += upFoot;
	else
		walkyy -= upFoot;
	
	theta beta = IK(0, walkyy - 3, 1.5f, 1.5f);;
	if (footFlag == 3 || footFlag == 4) {
		angle[6] = beta.theta1;
		angle[7] = beta.theta2;
	}
	else {
		angle[8] = beta.theta1;
		angle[9] = beta.theta2;
	}
}

//机器人立正
void stand() {
	for (int i = 0; i < 12; i++) {
		angle[i] = 0;
		IKAngle[i] = 0;
	}
	Langle = 0;
	walkyy = 0;
	xx = 0;
	yy = 0;
}


//处理所有输入：键盘交互
void processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		camera.ProcessKeyboard(FORWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		camera.ProcessKeyboard(BACKWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		camera.ProcessKeyboard(LEFT, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		camera.ProcessKeyboard(RIGHT, deltaTime);

	//运动设置 正向运动学
	if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS)
	{//左大臂和右大臂
		angle[2] += 0.05 * (float)glfwGetTime();
		angle[4] += 0.05 * (float)glfwGetTime();
	}
	if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS)
	{//左小臂和右小臂
		angle[3] += 0.05 * flag[3] * (float)glfwGetTime();
		angle[5] += 0.05 * flag[5] * (float)glfwGetTime();
		if (abs(angle[3]) > 45) {
			flag[3] = -flag[3];
			flag[5] = -flag[5];
		}
	}
	if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
	{//左大腿和右大腿
		angle[6] += 0.05 * flag[6] * (float)glfwGetTime();
		angle[8] += 0.05 * flag[8] * (float)glfwGetTime();
		if (abs(angle[6]) > 135) {
			flag[6] = -flag[6];
			flag[8] = -flag[8];
		}
	}
	if (glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS)
	{//左小腿和右小腿
		angle[7] += 0.05 * flag[7] * (float)glfwGetTime();
		angle[9] += 0.05 * flag[9] * (float)glfwGetTime();
		if (abs(angle[7]) > 45) {
			flag[7] = -flag[7];
			flag[9] = -flag[9];
		}
	}
	if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS)
	{//一段小动画

	}

	if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
	{//机器人转身
		Langle += 0.30 * (float)glfwGetTime();
	}

	if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS)
	{//逆向运动学
		yy += 0.005f;
		theta alpha = IK(xx, yy - 2, 1.0, 1.0);
		IKAngle[4] = alpha.theta1;
		IKAngle[5] = alpha.theta2;
	}
	if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS)
	{//逆向运动学
		xx += 0.005f;
		theta alpha = IK(xx, yy - 2, 1.0, 1.0);
		IKAngle[4] = alpha.theta1;
		IKAngle[5] = alpha.theta2;
	}
	if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS)
	{//逆向运动学
		yy -= 0.005f;
		theta alpha = IK(xx, yy - 2, 1.0, 1.0);
		IKAngle[4] = alpha.theta1;
		IKAngle[5] = alpha.theta2;
	}
	if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS)
	{//逆向运动学
		xx -= 0.005f;
		theta alpha = IK(xx, yy - 2, 1.0, 1.0);
		IKAngle[4] = alpha.theta1;
		IKAngle[5] = alpha.theta2;
	}
	//stand
	if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS) {
		stand();
	}
	//walk 行走
	if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS) {
		//stand();
		walk(1);
	}
	if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS) {
		//stand();
		walk(-1);
	}
}

//窗口大小设置
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}


//鼠标交互
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos; //颠倒Y坐标
	lastX = xpos;
	lastY = ypos;

	camera.ProcessMouseMovement(xoffset, yoffset);
}

// 鼠标滚动调用  下滚动 视角变大
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	camera.ProcessMouseScroll(yoffset);
}


// cube 的VAO绑定与纹理绑定
void cubeBind(const char * path)
{
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	//位置绑定
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	//法线绑定
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(2);
    //纹理绑定
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
	glEnableVertexAttribArray(1);

	glGenTextures(0, &texture1);
	glBindTexture(GL_TEXTURE_2D, texture1);
	//设置纹理环绕参数
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	//设置纹理过滤参数
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	//加载纹理
	int width, height, nrChannels;
	stbi_set_flip_vertically_on_load(true);
	unsigned char *data = stbi_load(path, &width, &height, &nrChannels, 0);
	if (data)
	{
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
		glGenerateMipmap(GL_TEXTURE_2D);
	}
	else
	{
		cout << "Failed to load texture" << endl;
	}
	stbi_image_free(data);

}


//加载天空盒子图片
unsigned int loadCubemap(vector<std::string> faces)
{
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