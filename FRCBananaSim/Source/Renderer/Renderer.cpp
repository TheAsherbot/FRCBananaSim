/**
 * @file Renderer.cpp
 * @brief Implementation of OpenGL 3.3 Core rendering backend and primitive buffers.
 */

#include "Renderer/Renderer.h"
#include "Renderer/Components/MeshComponent.h"
#include "Core/ECS/Components/TransformComponent.h"
#include "Telemetry/TelemetryMacros.h"
#include "Utils/DebugDraw.h"
#include "Utils/Log.h"

 // GLAD must be included before GLFW to prevent header redefinition conflicts
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <imgui/imgui.h>
#include <imgui/backend/imgui_impl_glfw.h>
#include <imgui/backend/imgui_impl_opengl3.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <cmath>
#include <cstddef>
#include <unordered_map>
#include <vector>

namespace FRCBananaSim::Renderer
{
    using Core::ECS::Components::WorldTransformComponent;
    using Components::MeshComponent;
    using Components::PrimitiveShape;
    using Utils::DebugDraw;
    using Utils::DebugLineVertex;
    using Utils::DebugDrawData;

    namespace
    {
        constexpr float kPi = 3.14159265358979323846f;

        const char* kMeshVertexSrc = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;

out vec3 vNormal;
out vec3 vFragPos;

void main()
{
    vFragPos = vec3(uModel * vec4(aPos, 1.0));
    vNormal = mat3(transpose(inverse(uModel))) * aNormal;
    gl_Position = uProjection * uView * vec4(vFragPos, 1.0);
}
)";

        const char* kMeshFragmentSrc = R"(
#version 330 core
in vec3 vNormal;
in vec3 vFragPos;

uniform vec3 uColor;
uniform vec3 uLightDir;

out vec4 FragColor;

void main()
{
    vec3 normal = normalize(vNormal);
    float diff = max(dot(normal, -uLightDir), 0.0);
    vec3 ambient = 0.3 * uColor;
    vec3 diffuse = 0.7 * diff * uColor;
    FragColor = vec4(ambient + diffuse, 1.0);
}
)";

        const char* kDebugLineVertexSrc = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

uniform mat4 uView;
uniform mat4 uProjection;

out vec3 vColor;

void main()
{
    vColor = aColor;
    gl_Position = uProjection * uView * vec4(aPos, 1.0);
}
)";

        const char* kDebugLineFragmentSrc = R"(
#version 330 core
in vec3 vColor;
out vec4 FragColor;

void main()
{
    FragColor = vec4(vColor, 1.0);
}
)";

        struct MeshVertex
        {
            glm::vec3 position;
            glm::vec3 normal;
        };

        std::vector<MeshVertex> GenerateCube()
        {
            std::vector<MeshVertex> v;
            auto face = [&](glm::vec3 n, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d)
                {
                    v.push_back({ a, n }); v.push_back({ b, n }); v.push_back({ c, n });
                    v.push_back({ a, n }); v.push_back({ c, n }); v.push_back({ d, n });
                };
            face({ 0,0,1 }, { -0.5f,-0.5f,0.5f }, { 0.5f,-0.5f,0.5f }, { 0.5f,0.5f,0.5f }, { -0.5f,0.5f,0.5f });
            face({ 0,0,-1 }, { 0.5f,-0.5f,-0.5f }, { -0.5f,-0.5f,-0.5f }, { -0.5f,0.5f,-0.5f }, { 0.5f,0.5f,-0.5f });
            face({ 1,0,0 }, { 0.5f,-0.5f,0.5f }, { 0.5f,-0.5f,-0.5f }, { 0.5f,0.5f,-0.5f }, { 0.5f,0.5f,0.5f });
            face({ -1,0,0 }, { -0.5f,-0.5f,-0.5f }, { -0.5f,-0.5f,0.5f }, { -0.5f,0.5f,0.5f }, { -0.5f,0.5f,-0.5f });
            face({ 0,1,0 }, { -0.5f,0.5f,0.5f }, { 0.5f,0.5f,0.5f }, { 0.5f,0.5f,-0.5f }, { -0.5f,0.5f,-0.5f });
            face({ 0,-1,0 }, { -0.5f,-0.5f,-0.5f }, { 0.5f,-0.5f,-0.5f }, { 0.5f,0.5f,0.5f }, { -0.5f,-0.5f,0.5f });
            return v;
        }

        std::vector<MeshVertex> GeneratePlane()
        {
            std::vector<MeshVertex> v;
            glm::vec3 n(0.0f, 1.0f, 0.0f);
            v.push_back({ {-0.5f, 0.0f, -0.5f}, n });
            v.push_back({ { 0.5f, 0.0f,  0.5f}, n });
            v.push_back({ { 0.5f, 0.0f, -0.5f}, n });
            v.push_back({ {-0.5f, 0.0f, -0.5f}, n });
            v.push_back({ {-0.5f, 0.0f,  0.5f}, n });
            v.push_back({ { 0.5f, 0.0f,  0.5f}, n });
            return v;
        }

        std::vector<MeshVertex> GenerateSphere(int rings = 16, int segments = 24)
        {
            std::vector<MeshVertex> v;
            const float radius = 0.5f;

            for (int i = 0; i < rings; ++i)
            {
                float lat0 = kPi * (-0.5f + static_cast<float>(i) / rings);
                float lat1 = kPi * (-0.5f + static_cast<float>(i + 1) / rings);
                float y0 = std::sin(lat0), r0 = std::cos(lat0);
                float y1 = std::sin(lat1), r1 = std::cos(lat1);

                for (int j = 0; j < segments; ++j)
                {
                    float lon0 = 2.0f * kPi * static_cast<float>(j) / segments;
                    float lon1 = 2.0f * kPi * static_cast<float>(j + 1) / segments;

                    glm::vec3 p00(r0 * std::cos(lon0), y0, r0 * std::sin(lon0));
                    glm::vec3 p01(r0 * std::cos(lon1), y0, r0 * std::sin(lon1));
                    glm::vec3 p10(r1 * std::cos(lon0), y1, r1 * std::sin(lon0));
                    glm::vec3 p11(r1 * std::cos(lon1), y1, r1 * std::sin(lon1));

                    v.push_back({ p00 * radius, glm::normalize(p00) });
                    v.push_back({ p10 * radius, glm::normalize(p10) });
                    v.push_back({ p11 * radius, glm::normalize(p11) });

                    v.push_back({ p00 * radius, glm::normalize(p00) });
                    v.push_back({ p11 * radius, glm::normalize(p11) });
                    v.push_back({ p01 * radius, glm::normalize(p01) });
                }
            }
            return v;
        }

        std::vector<MeshVertex> GenerateCylinderY(int segments = 24)
        {
            std::vector<MeshVertex> v;
            const float radius = 0.5f;
            const float halfHeight = 0.5f;

            for (int j = 0; j < segments; ++j)
            {
                float theta0 = 2.0f * kPi * static_cast<float>(j) / segments;
                float theta1 = 2.0f * kPi * static_cast<float>(j + 1) / segments;

                float c0 = std::cos(theta0), s0 = std::sin(theta0);
                float c1 = std::cos(theta1), s1 = std::sin(theta1);

                glm::vec3 n0(c0, 0.0f, s0);
                glm::vec3 n1(c1, 0.0f, s1);

                glm::vec3 p00(c0 * radius, -halfHeight, s0 * radius);
                glm::vec3 p01(c0 * radius, halfHeight, s0 * radius);
                glm::vec3 p10(c1 * radius, -halfHeight, s1 * radius);
                glm::vec3 p11(c1 * radius, halfHeight, s1 * radius);

                // Cylinder wall
                v.push_back({ p00, n0 });
                v.push_back({ p10, n1 });
                v.push_back({ p11, n1 });

                v.push_back({ p00, n0 });
                v.push_back({ p11, n1 });
                v.push_back({ p01, n0 });

                // Top cap (+Y)
                v.push_back({ { 0.0f, halfHeight, 0.0f }, { 0.0f, 1.0f, 0.0f } });
                v.push_back({ p01, { 0.0f, 1.0f, 0.0f } });
                v.push_back({ p11, { 0.0f, 1.0f, 0.0f } });

                // Bottom cap (-Y)
                v.push_back({ { 0.0f, -halfHeight, 0.0f }, { 0.0f, -1.0f, 0.0f } });
                v.push_back({ p10, { 0.0f, -1.0f, 0.0f } });
                v.push_back({ p00, { 0.0f, -1.0f, 0.0f } });
            }
            return v;
        }

        std::vector<MeshVertex> GenerateCylinderX(int segments = 24)
        {
            std::vector<MeshVertex> v;
            const float radius = 0.5f;
            const float halfWidth = 0.5f;

            for (int j = 0; j < segments; ++j)
            {
                float theta0 = 2.0f * kPi * static_cast<float>(j) / segments;
                float theta1 = 2.0f * kPi * static_cast<float>(j + 1) / segments;

                float c0 = std::cos(theta0), s0 = std::sin(theta0);
                float c1 = std::cos(theta1), s1 = std::sin(theta1);

                glm::vec3 n0(0.0f, c0, s0);
                glm::vec3 n1(0.0f, c1, s1);

                glm::vec3 p00(-halfWidth, c0 * radius, s0 * radius);
                glm::vec3 p01(halfWidth, c0 * radius, s0 * radius);
                glm::vec3 p10(-halfWidth, c1 * radius, s1 * radius);
                glm::vec3 p11(halfWidth, c1 * radius, s1 * radius);

                // Cylinder tread wall
                v.push_back({ p00, n0 });
                v.push_back({ p10, n1 });
                v.push_back({ p11, n1 });

                v.push_back({ p00, n0 });
                v.push_back({ p11, n1 });
                v.push_back({ p01, n0 });

                // Right cap (+X)
                v.push_back({ { halfWidth, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } });
                v.push_back({ p01, { 1.0f, 0.0f, 0.0f } });
                v.push_back({ p11, { 1.0f, 0.0f, 0.0f } });

                // Left cap (-X)
                v.push_back({ { -halfWidth, 0.0f, 0.0f }, { -1.0f, 0.0f, 0.0f } });
                v.push_back({ p10, { -1.0f, 0.0f, 0.0f } });
                v.push_back({ p00, { -1.0f, 0.0f, 0.0f } });
            }
            return v;
        }

        GLuint CompileShader(GLenum type, const char* src, const char* label)
        {
            GLuint shader = glCreateShader(type);
            glShaderSource(shader, 1, &src, nullptr);
            glCompileShader(shader);
            GLint success = 0;
            glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
            if (!success)
            {
                char log[1024];
                glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
                SIM_LOG_ERROR("Shader compile error (" << label << "): " << log);
            }
            return shader;
        }

        GLuint LinkProgram(GLuint vs, GLuint fs, const char* label)
        {
            GLuint program = glCreateProgram();
            glAttachShader(program, vs);
            glAttachShader(program, fs);
            glLinkProgram(program);
            GLint success = 0;
            glGetProgramiv(program, GL_LINK_STATUS, &success);
            if (!success)
            {
                char log[1024];
                glGetProgramInfoLog(program, sizeof(log), nullptr, log);
                SIM_LOG_ERROR("Shader link error (" << label << "): " << log);
            }
            return program;
        }

        struct PrimitiveMesh
        {
            GLuint vao = 0;
            GLuint vbo = 0;
            GLsizei vertexCount = 0;
        };

        PrimitiveMesh UploadMesh(const std::vector<MeshVertex>& vertices)
        {
            PrimitiveMesh mesh;
            mesh.vertexCount = static_cast<GLsizei>(vertices.size());
            glGenVertexArrays(1, &mesh.vao);
            glBindVertexArray(mesh.vao);
            glGenBuffers(1, &mesh.vbo);
            glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
            glBufferData(GL_ARRAY_BUFFER,
                static_cast<GLsizeiptr>(vertices.size() * sizeof(MeshVertex)),
                vertices.data(), GL_STATIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex),
                reinterpret_cast<void*>(offsetof(MeshVertex, position)));
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex),
                reinterpret_cast<void*>(offsetof(MeshVertex, normal)));
            glEnableVertexAttribArray(1);
            glBindVertexArray(0);
            return mesh;
        }

        void FramebufferSizeCallback(GLFWwindow*, int width, int height)
        {
            glViewport(0, 0, width, height);
        }
    } // namespace

    struct Renderer::Impl
    {
        GLFWwindow* window = nullptr;
        bool valid = false;

        GLuint meshProgram = 0;
        GLuint debugLineProgram = 0;
        std::unordered_map<PrimitiveShape, PrimitiveMesh> primitives;

        GLuint debugLineVao = 0;
        GLuint debugLineVbo = 0;

        Camera camera;
        Input input;
        double lastFrameTime = 0.0;
    };

    Renderer::Renderer(int width, int height, const char* title)
        : impl(std::make_unique<Impl>())
    {
        if (!glfwInit())
        {
            SIM_LOG_FATAL("glfwInit() failed");
            return;
        }

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);

        impl->window = glfwCreateWindow(width, height, title, nullptr, nullptr);
        if (!impl->window)
        {
            SIM_LOG_FATAL("glfwCreateWindow() failed - check driver/GL 3.3 core support");
            glfwTerminate();
            return;
        }

        glfwMakeContextCurrent(impl->window);
        glfwSwapInterval(0); // Disable VSync by default so software frame limiter has full control
        glfwSetFramebufferSizeCallback(impl->window, FramebufferSizeCallback);

        impl->input.AttachWindow(impl->window);

        if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
        {
            SIM_LOG_FATAL("gladLoadGLLoader() failed - could not load OpenGL function pointers");
            glfwDestroyWindow(impl->window);
            impl->window = nullptr;
            glfwTerminate();
            return;
        }
        SIM_LOG_INFO("OpenGL " << GLVersion.major << "." << GLVersion.minor << " loaded");

        glEnable(GL_DEPTH_TEST);
        glViewport(0, 0, width, height);

        GLuint meshVS = CompileShader(GL_VERTEX_SHADER, kMeshVertexSrc, "mesh vertex");
        GLuint meshFS = CompileShader(GL_FRAGMENT_SHADER, kMeshFragmentSrc, "mesh fragment");
        impl->meshProgram = LinkProgram(meshVS, meshFS, "mesh");
        glDeleteShader(meshVS);
        glDeleteShader(meshFS);

        GLuint lineVS = CompileShader(GL_VERTEX_SHADER, kDebugLineVertexSrc, "debug line vertex");
        GLuint lineFS = CompileShader(GL_FRAGMENT_SHADER, kDebugLineFragmentSrc, "debug line fragment");
        impl->debugLineProgram = LinkProgram(lineVS, lineFS, "debug line");
        glDeleteShader(lineVS);
        glDeleteShader(lineFS);

        impl->primitives[PrimitiveShape::Cube] = UploadMesh(GenerateCube());
        impl->primitives[PrimitiveShape::Sphere] = UploadMesh(GenerateSphere());
        impl->primitives[PrimitiveShape::Plane] = UploadMesh(GeneratePlane());
        impl->primitives[PrimitiveShape::Cylinder] = UploadMesh(GenerateCylinderY());
        impl->primitives[PrimitiveShape::CylinderX] = UploadMesh(GenerateCylinderX());

        glGenVertexArrays(1, &impl->debugLineVao);
        glGenBuffers(1, &impl->debugLineVbo);

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

        ImGui::StyleColorsDark();
        ImGui_ImplGlfw_InitForOpenGL(impl->window, true);
        ImGui_ImplOpenGL3_Init("#version 330 core");

        impl->valid = true;
        impl->lastFrameTime = glfwGetTime();
    }

    Renderer::~Renderer()
    {
        if (impl->valid)
        {
            ImGui_ImplOpenGL3_Shutdown();
            ImGui_ImplGlfw_Shutdown();
            ImGui::DestroyContext();

            glDeleteProgram(impl->meshProgram);
            glDeleteProgram(impl->debugLineProgram);

            for (auto& [shape, mesh] : impl->primitives)
            {
                glDeleteVertexArrays(1, &mesh.vao);
                glDeleteBuffers(1, &mesh.vbo);
            }

            glDeleteVertexArrays(1, &impl->debugLineVao);
            glDeleteBuffers(1, &impl->debugLineVbo);

            glfwDestroyWindow(impl->window);
            glfwTerminate();
        }
    }

    bool Renderer::IsValid() const
    {
        return impl->valid;
    }

    bool Renderer::ShouldClose() const
    {
        return !impl->valid || glfwWindowShouldClose(impl->window);
    }

    const GLFWwindow* Renderer::GetWindow() const
    {
        return impl->window;
    }

    Camera& Renderer::GetCamera()
    {
        return impl->camera;
    }

    const Camera& Renderer::GetCamera() const
    {
        return impl->camera;
    }

    Input& Renderer::GetInput()
    {
        return impl->input;
    }

    const Input& Renderer::GetInput() const
    {
        return impl->input;
    }

    GLFWwindow* Renderer::GetWindow()
    {
        return impl->window;
    }

    void Renderer::BeginFrame()
    {
        if (!impl->valid)
        {
            return;
        }

        glfwPollEvents();
        impl->input.Update();

        // Right-click toggles free-look cursor lock for Camera
        if (impl->input.WasKeyPressed(GLFW_KEY_F1))
        {
            impl->input.SetCursorCaptured(!impl->input.IsCursorCaptured());
        }

        double now = glfwGetTime();
        float dt = static_cast<float>(now - impl->lastFrameTime);
        impl->lastFrameTime = now;
        impl->camera.ProcessInput(impl->input, dt);

        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
    }

    void Renderer::DrawMesh(const MeshComponent& mesh, const WorldTransformComponent& worldTransform)
    {
        if (!impl->valid)
        {
            return;
        }

        auto it = impl->primitives.find(mesh.shape);
        if (it == impl->primitives.end())
        {
            return;
        }

        glm::mat4 model = glm::translate(glm::mat4(1.0f), worldTransform.worldPosition)
            * glm::mat4_cast(worldTransform.worldRotation)
            * glm::scale(glm::mat4(1.0f), worldTransform.worldScale);

        int fbWidth = 1, fbHeight = 1;
        glfwGetFramebufferSize(impl->window, &fbWidth, &fbHeight);
        float aspect = fbHeight > 0 ? static_cast<float>(fbWidth) / static_cast<float>(fbHeight) : 1.0f;

        glUseProgram(impl->meshProgram);
        glUniformMatrix4fv(glGetUniformLocation(impl->meshProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(impl->meshProgram, "uView"), 1, GL_FALSE,
            glm::value_ptr(impl->camera.GetViewMatrix()));
        glUniformMatrix4fv(glGetUniformLocation(impl->meshProgram, "uProjection"), 1, GL_FALSE,
            glm::value_ptr(impl->camera.GetProjectionMatrix(aspect)));
        glUniform3fv(glGetUniformLocation(impl->meshProgram, "uColor"), 1, glm::value_ptr(mesh.color));
        glm::vec3 lightDir = glm::normalize(glm::vec3(-0.4f, -1.0f, -0.3f));
        glUniform3fv(glGetUniformLocation(impl->meshProgram, "uLightDir"), 1, glm::value_ptr(lightDir));

        glBindVertexArray(it->second.vao);
        glDrawArrays(GL_TRIANGLES, 0, it->second.vertexCount);
    }

    void Renderer::FlushDebugDraw()
    {
        if (!impl->valid)
        {
            return;
        }

        DebugDrawData debugData = DebugDraw::TakeLines();
        if (debugData.depthTestedLines.empty() && debugData.alwaysOnTopLines.empty())
        {
            return;
        }

        int fbWidth = 1, fbHeight = 1;
        glfwGetFramebufferSize(impl->window, &fbWidth, &fbHeight);
        float aspect = fbHeight > 0 ? static_cast<float>(fbWidth) / static_cast<float>(fbHeight) : 1.0f;

        glUseProgram(impl->debugLineProgram);
        glUniformMatrix4fv(glGetUniformLocation(impl->debugLineProgram, "uView"), 1, GL_FALSE,
            glm::value_ptr(impl->camera.GetViewMatrix()));
        glUniformMatrix4fv(glGetUniformLocation(impl->debugLineProgram, "uProjection"), 1, GL_FALSE,
            glm::value_ptr(impl->camera.GetProjectionMatrix(aspect)));

        glBindVertexArray(impl->debugLineVao);
        glBindBuffer(GL_ARRAY_BUFFER, impl->debugLineVbo);

        const GLsizei depthTestedCount = static_cast<GLsizei>(debugData.depthTestedLines.size());
        const GLsizei alwaysOnTopCount = static_cast<GLsizei>(debugData.alwaysOnTopLines.size());

        debugData.depthTestedLines.insert(
            debugData.depthTestedLines.end(),
            debugData.alwaysOnTopLines.begin(),
            debugData.alwaysOnTopLines.end());

        glBufferData(GL_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(debugData.depthTestedLines.size() * sizeof(DebugLineVertex)),
            debugData.depthTestedLines.data(), GL_DYNAMIC_DRAW);

        // Configure vertex attributes (Location 0: Position, Location 1: Color)
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(DebugLineVertex),
            reinterpret_cast<void*>(offsetof(DebugLineVertex, position)));

        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(DebugLineVertex),
            reinterpret_cast<void*>(offsetof(DebugLineVertex, color)));

        if (depthTestedCount > 0)
        {
            glEnable(GL_DEPTH_TEST);
            glDrawArrays(GL_LINES, 0, depthTestedCount);
        }

        if (alwaysOnTopCount > 0)
        {
            glDisable(GL_DEPTH_TEST);
            glDrawArrays(GL_LINES, depthTestedCount, alwaysOnTopCount);
            glEnable(GL_DEPTH_TEST);
        }

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void Renderer::EndFrame()
    {
        if (!impl->valid)
        {
            return;
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(impl->window);
    }
}