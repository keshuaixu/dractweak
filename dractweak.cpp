#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>

#include <GL/gl3w.h>
#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <vector>

#include <Amp1394/AmpIORevision.h>
#include "PortFactory.h"
#include "AmpIO.h"
#include "Amp1394Time.h"

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

float current_from_adc_count(int count) {
    return (count - 0x8000) / 4000.0;
}


int dac_count_from_current(float current) {
    int result = int(current * 4000) + 0x8000;
    return std::clamp(result, 0, 0xffff);
}

uint16_t pi_fixed_from_float(float val) {
    int result = int(val * 4096);
    return std::clamp(result, 0, 0xffff);
}

const char* current_format = "% .03f A";
const float current_max = 4.5; 

AmpIO *board;
BasePort *Port;
int BoardId = 0;
float Imax = current_from_adc_count(0xffff);

int axis = 1;
float i_setp;
int mode;

int float_to_fixed = 4096;
float kp, ki;
int ilim, olim;

void draw_poke_window() {
    ImGui::Begin("No broadcast",0);
    if (ImGui::Button("reboot")) {
        board->WriteReboot();
    }
    // ImGui::SliderInt("axis", &axis, 1, 10);
    for (int i = 1; i < 11; i ++) {
        char button_name[16];
        sprintf(button_name, "%d", i);
        if (ImGui::RadioButton(button_name, i == axis)) {
            axis = i;
        }
        if (i < 10) {
            ImGui::SameLine();
        }
    }
    ImGui::Separator();

    quadlet_t current_q;
    Port->ReadQuadlet(BoardId, (axis) << 4 | 0x00, current_q);
    uint16_t current = static_cast<uint16_t>(current_q & 0xffff);
    ImGui::LabelText("I meas hex", "0x%04X", current);
    float im = current_from_adc_count(current);
    ImGui::LabelText("I meas", current_format, im);
    ImGui::Separator();
    ImGui::LabelText("duty cycle", "%d", board->ReadDutyCycle(axis - 1));
    uint32_t fault = board->ReadFault(axis - 1);
    ImGui::LabelText("fault", "%X", fault);
    quadlet_t debug;
    Port->ReadQuadlet(BoardId, (axis) << 4 | 0x0f | 0x9000, debug);
    ImGui::LabelText("debug", "0x%08X", debug);
    // printf("0x%08X ", debug);
    ImGui::RadioButton("overheat", fault & 1 << 3);
    ImGui::SameLine();
    ImGui::RadioButton("overcurrent", fault & 1 << 2);
    ImGui::SameLine();   
    ImGui::RadioButton("regulation", fault & 1 << 1);
    ImGui::SameLine();
    ImGui::RadioButton("adc", fault & 1 << 0);
    ImGui::Separator();

    quadlet_t i_setp_q;
    Port->ReadQuadlet(BoardId, (axis) << 4 | 0x01, i_setp_q);
    ImGui::LabelText("setp hex", "0x%04X", i_setp_q);
    i_setp = current_from_adc_count(i_setp_q);
    ImGui::DragFloat("setp", &i_setp, 0.001, -current_max, current_max, current_format);
    bool setp_edited = ImGui::IsItemEdited();
    // uint32_t zero = 0;
    // uint32_t ffff = 0xffff;
    // ImGui::InputScalar("setp hex",ImGuiDataType_U32 ,&i_setp_q,(uint32_t) 1, "0x%04X");
    std::vector<float> current_presets = { -3, -2, -1, -0.1, 0, 0.1, 1, 2, 3};
    char buf[10];
    for (auto p : current_presets) {
        sprintf(buf, "%.1g", p);
        if (ImGui::Button(buf, ImVec2(40, 0))) {
            i_setp = p;
            setp_edited = true;
        }
        ImGui::SameLine();
    }
    ImGui::Text(" ");
    i_setp_q = dac_count_from_current(i_setp);

    if (setp_edited) {
        Port->WriteQuadlet(BoardId, (axis) << 4 | 0x01, i_setp_q);
    }
    ImGui::Separator();

    int mode_read = board->ReadMotorControlMode(axis - 1);
    if (ImGui::RadioButton("reset", mode_read == AmpIO::MotorControlMode::RESET)) {
        if (ImGui::IsItemEdited()) {
            board->WriteMotorControlMode(axis - 1, AmpIO::MotorControlMode::RESET);
        }
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("voltage", mode_read == AmpIO::MotorControlMode::VOLTAGE)) {
        if (ImGui::IsItemEdited()) {
            board->WriteMotorControlMode(axis - 1, AmpIO::MotorControlMode::VOLTAGE);
        }
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("current", mode_read == AmpIO::MotorControlMode::CURRENT)) {
        if (ImGui::IsItemEdited()) {
            board->WriteMotorControlMode(axis - 1, AmpIO::MotorControlMode::CURRENT);
        }
    }

    if (true || ImGui::Button("read")) {
        kp = (float) board->ReadCurrentKpRaw(axis - 1) / float_to_fixed;
        ki = (float) board->ReadCurrentKiRaw(axis - 1) / float_to_fixed;
        ilim = board->ReadCurrentITermLimitRaw(axis - 1);
        olim = board->ReadDutyCycleLimit(axis - 1);
    }
    if (ImGui::Button("load default")) {
        kp = 0.001;
        ki = 0.001;
        ilim = 200;
        olim = 1020;
        board->WriteCurrentKpRaw(axis - 1, pi_fixed_from_float(kp));
        board->WriteCurrentKiRaw(axis - 1, pi_fixed_from_float(ki));
        board->WriteCurrentITermLimitRaw(axis - 1, ilim);
        board->WriteDutyCycleLimit(axis - 1, olim);
    }
    ImGui::DragFloat("kp", &kp, 0.0002, 0, 2, "%.4f");
    if (ImGui::IsItemEdited()) {
        board->WriteCurrentKpRaw(axis - 1, pi_fixed_from_float(kp));
    }
    ImGui::DragFloat("ki", &ki, 0.0002, 0, 2, "%.4f");
    if (ImGui::IsItemEdited()) {
        board->WriteCurrentKiRaw(axis - 1, pi_fixed_from_float(ki));
    }
    ImGui::InputInt("i term lim", &ilim, 1, 10);
    if (ImGui::IsItemEdited()) {
        board->WriteCurrentITermLimitRaw(axis - 1, ilim);
    }
    ImGui::InputInt("duty cycle lim", &olim, 1, 10);    
        if (ImGui::IsItemEdited()) {
        board->WriteDutyCycleLimit(axis - 1, olim);
    }

    ImGui::End();
}

bool collect_cb(quadlet_t* data, unsigned short num_avail) {
    return true;
}


int main(int, char**)
{
    // Setup window
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only

    // Create window with graphics context
    GLFWwindow* window = glfwCreateWindow(1440, 720, "dRAC tweak", NULL, NULL);
    if (window == NULL)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync
    bool err = gl3wInit() != 0;
    if (err)
    {
        fprintf(stderr, "Failed to initialize OpenGL loader!\n");
        return 1;
    }

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Load Fonts
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
    // - If the file cannot be loaded, the function will return NULL. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
    // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
    // - Read 'docs/FONTS.md' for more instructions and details.
    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
    //io.Fonts->AddFontDefault();
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/ProggyTiny.ttf", 10.0f);
    //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesJapanese());
    //IM_ASSERT(font != NULL);

    // Our state
    bool show_demo_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    BasePort::ProtocolType protocol = BasePort::PROTOCOL_SEQ_RW;
    std::vector<AmpIO*> BoardList;
    std::vector<AmpIO_UInt32> BoardStatusList;

    // measure time between reads
    AmpIO_UInt32 maxTime = 0;
    AmpIO_UInt32 lastTime = 0;

    unsigned int curAxis = 0;     // Current axis (0=all, 1-8)
    unsigned int curBoardIndex = 0;
    unsigned int curAxisIndex = 0;
    char axisString[4] = "all";
    std::string portDescription;
    board = new AmpIO(BoardId);
    Port = PortFactory(portDescription.c_str(), std::cout);
    Port->AddBoard(board);

    int num_axes = 10;
    int motor_current_read[num_axes];
    float current_setpoint[num_axes] = {0.0};


    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        glfwPollEvents();
        Port->ReadAllBoards();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();



        ImGui::SetNextWindowPos( ImVec2(0,0) );
        int glfw_window_width, glfw_window_height;
        glfwGetWindowSize(window, &glfw_window_width, &glfw_window_height);
        // ImGui::SetNextWindowSize(ImVec2(glfw_window_width, glfw_window_height));
        // ImGui::Begin("dRAC",0 , ImGuiWindowFlags_HorizontalScrollbar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoTitleBar);
        ImGui::Begin("dRAC",0);
        if (ImGui::TreeNodeEx("board", ImGuiTreeNodeFlags_DefaultOpen)){
            ImGui::BeginTable("boardtable", 5);
            ImGui::TableNextColumn();
            // ImGui::AlignTextToFramePadding();
            if (ImGui::Button("reboot")) {
                board->WriteReboot();
            }
            bool power_enable = board->GetPowerEnable(); //board->GetPowerStatus();
            if (ImGui::Checkbox("MV", &power_enable)) {
                if (ImGui::IsItemEdited()) {
                    board->SetPowerEnable(power_enable);
                }
            }
            static bool relay_enable = false;
            if (ImGui::Checkbox("Relay", &relay_enable)) {
                if (ImGui::IsItemEdited()) {
                    board->SetSafetyRelay(relay_enable);
                }
            }
            ImGui::LabelText("48V voltage", "%.2f", 0);
            // ImGui::Text("motor power = %s", board->GetPowerEnable() ? "on" : "off");
            ImGui::TableNextColumn();
            ImGui::TableNextColumn();
            // ImGui::SameLine();
            ImGui::EndTable();

            ImGui::TreePop();

        }
        for (int axis_index = 0; axis_index < num_axes; axis_index++) {
            char window_name[32];
            sprintf(window_name, "Axis %d", axis_index + 1);
            // ImGui::BeginChild(window_name, ImVec2(200, 0), true, ImGuiWindowFlags_None);
            if (ImGui::TreeNodeEx(window_name, ImGuiTreeNodeFlags_DefaultOpen)){
                ImGui::BeginTable("ax", 4);
                ImGui::TableNextColumn();
                bool axis_enable = board->GetAmpEnable(axis_index);
                // printf("axis %d en %d\n", axis_index, axis_enable);
                if (ImGui::Checkbox("Enable", &axis_enable)) {
                    if (ImGui::IsItemEdited()) {
                        board->SetAmpEnable(axis_index, axis_enable);
                    }
                }

                ImGui::TableNextColumn();


                motor_current_read[axis_index] = board->GetMotorCurrent(axis_index);
                // ImGui::DragInt("Im", &(motor_current_read[axis_index]),0.1f, 0, 0xffff,"0x%04X");
                // ImGui::BulletText("Im");
                ImGui::LabelText("adc", "0x%04X", motor_current_read[axis_index]);
                float im = current_from_adc_count(motor_current_read[axis_index]);
                ImGui::LabelText("I", current_format, im);
                // ImGui::ProgressBar(std::abs(im)/Imax,ImVec2(0.0f, 0.0f),"");
                // ImGui::Text("0x%04X", motor_current_read[axis_index]);

                ImGui::DragFloat("I setp", &(current_setpoint[axis_index]), 0.001, -current_max, current_max, current_format);
                auto i_setp_q = dac_count_from_current(i_setp);
                if (ImGui::IsItemEdited()) {
                    board->SetMotorCurrent(axis_index, i_setp_q);
                }
                ImGui::TableNextColumn();

                if (ImGui::Button("Step")) {
                    board->DataCollectionStart(axis_index + 1);
                    board->SetMotorCurrent(axis_index, 0x8100);
                    Port->WriteAllBoards();
                    Amp1394_Sleep(0.100);
                    board->SetMotorCurrent(axis_index, 0x8000);
                    Port->WriteAllBoards();
                }

                ImGui::EndTable();
                ImGui::TreePop();
            }
            // ImGui::Text("%.04f A", im);
            // ImGui::EndChild();
            // ImGui::SameLine();
        }

        ImGui::End();

        draw_poke_window();
        Port->WriteAllBoards();


        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);

    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
