#include "TestsWithGL_pcp.h"

#include <exception>
#include <string>

#include "ShaderProgram.h"
#include "DrawTriangleMesh.h"

#include "test_sim_core.h"

// call back function
static void framebuffer_size_callback(
	GLFWwindow *window, GLsizei width, GLsizei height)
{
	GLsizei win_size;
	GLsizei win_padding;
	if (width < height)
	{
		win_size = width;
		win_padding = (height - width) / 2;
		glViewport(0, win_padding, win_size, win_size);
	}
	else
	{
		win_size = height;
		win_padding = (width - height) / 2;
		glViewport(win_padding, 0, win_size, win_size);
	}
}

int display_triangle_mesh(TriangleMesh &tri_mesh, bool disp_tri_mesh, bool dis_bl, bool disp_bg_grid,
						  TriangleMesh::Edge *_edge, Point *_pt)
{
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	const int win_size = 600;
	GLFWwindow *window = glfwCreateWindow(win_size, win_size, "test_opengl", nullptr, nullptr);
	if (window == nullptr)
	{
		std::cout << "Failed to create GLFW window.\n";
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD.\n";
		glfwTerminate();
		return -1;
	}

	glViewport(0, 0, win_size, win_size);
	
	ShaderProgram shader;
	shader.init_from_file("..\\..\\Asset\\rb_vshader.txt",
						  "..\\..\\Asset\\rb_fshader.txt");
	size_t proj_mat_id = shader.init_from_code();


	DrawTriangleMesh draw_tri_mesh;
	draw_tri_mesh.init(tri_mesh);
	// draw point and line
	if (_edge) draw_tri_mesh.init_line_and_point(tri_mesh, *_edge, *_pt);
	
	// set projection matrix
	Rect disp_range = tri_mesh.get_display_range();
	GLfloat range = (GLfloat)(disp_range.xu > disp_range.yu ? disp_range.xu : disp_range.yu);
	GLfloat padding = range * 0.05f;
	glm::mat4 proj_mat = glm::ortho(-padding, range + padding, -padding, range + padding);
	shader.use();
	GLint proj_mat_id = shader.init_uniform("proj_mat");
	shader.set_uniform_matrix4f(proj_mat_id, glm::value_ptr(proj_mat));

	while (!glfwWindowShouldClose(window))
	{
		// set background color
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		shader.use();
		draw_tri_mesh.draw(disp_tri_mesh, dis_bl, disp_bg_grid);
		if (_edge) draw_tri_mesh.draw_line_and_point();
		
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwTerminate();
	return 0;
}
