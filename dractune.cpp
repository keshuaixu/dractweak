#define IMGUI_IMPL_OPENGL_LOADER_GLEW
#include <Amp1394/AmpIORevision.h>
#include "PortFactory.h"
#include "AmpIO.h"
#include "Amp1394Time.h"
#include "EthBasePort.h"
#include <chrono>
#include <thread>
#include <iomanip>
#include "CRC.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_stdlib.h"
#include <stdio.h>
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include<mutex>
#include <semaphore>
#include <cmath>
#include <array>
#include <fstream>
#include <filesystem>
#include "implot.h"


const std::string version = "1.0.0";
const auto mv_bit_to_volt = 1.31255e-3;
int command_wait_time = 20;

std::stringstream ss_console;

std::mutex port_mutex;
std::binary_semaphore rt_rw_sem{0};
bool testing = false;

void sleep(int t, bool wait_for_rt=true) {
    if (wait_for_rt)
        rt_rw_sem.acquire();    
    std::this_thread::sleep_for(std::chrono::milliseconds(t));
    if (wait_for_rt)
        rt_rw_sem.acquire();
}

void pass(std::string s="") {
    std::cout << std::setw(13) << "\U0001f7e2 Pass" << std::setw(30) << s << std::endl;
}

void fail(std::string s="") {
    std::cout << std::setw(13) << "\U0001F4A9 Fail" << std::setw(30) << s << std::endl;
}

void test(std::string s) {
    std::cout << std::setw(20) << s;
}


const auto red = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
const auto green = ImVec4(0.0f, 1.0f, 0.0f, 1.0f);
const auto white = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
ImU32 cell_bg_pass;
ImU32 cell_bg_fail;
ImU32 cell_bg_in_progress;
ImU32 cell_bg_untested;



static AmpIO* board;
static BasePort* Port;
static int board_id = 0;
static bool board_connected = false;
const int num_axes = 10;
int motor_current_read[num_axes];
float motor_voltage_read[num_axes];
int amp_fault_codes[num_axes];
float mv = 12.0;
const float amps_to_bits[10] = {4800.0, 4800.0, 16000.0, 16000.0, 16000.0, 16000.0, 16000.0, 16000.0, 16000.0, 16000.0};
const float internal_res_hr = 0.40;
const float internal_res_lr = 0.46;
const float internal_resistance[10] = {internal_res_hr,internal_res_hr,internal_res_lr,internal_res_lr,internal_res_lr,internal_res_lr,internal_res_lr,internal_res_lr,internal_res_lr,internal_res_lr};
const char* channel_names[10] = {"M1", "M2", "M3", "M4", "M5", "M6", "M7", "B1", "B2", "B3"};
const char* amp_fault_text[16] = {"-", "ADC saturated", "Current deviation", "HW overcurrent", "HW overtemp", "Undefined", "Undefined", "Undefined", "Undefined", "Undefined", "Undefined", "Undefined", "Undefined", "Undefined", "Undefined", "Undefined"};
static std::string BoardSN;
static std::string BoardSNRead;

int tuning_pulse_width = 100;
std::vector<float> current_history;
std::vector<float> current_history_t;

int tuning_axis_index = 0;
int kp, ki;
int ilim, olim;



void read_quadlet_threadsafe(unsigned char board_id, nodeaddr_t address, quadlet_t &data) {
    const std::lock_guard<std::mutex> lock(port_mutex);
    Port->ReadQuadlet(board_id, address, data);
}

void write_quadlet_threadsafe(unsigned char board_id, nodeaddr_t address, quadlet_t data) {
    const std::lock_guard<std::mutex> lock(port_mutex);
    Port->WriteQuadlet(board_id, address, data);
}

void update_all_boards() {
    static int consecutive_failures = 0;
    while (true) {
        if (board_connected){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            const std::lock_guard<std::mutex> lock(port_mutex);
            if (Port && Port->IsOK()) {
                auto w_result = Port->WriteAllBoards();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                auto r_result = Port->ReadAllBoards();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
           
                if (w_result == false || r_result == false) {
                    consecutive_failures++;
                    std::cerr << "eth fail" << std::endl;

                    if (consecutive_failures > 200) {
                        board_connected = false;
                        std::cerr << "Board disconnected" << std::endl;
                    }
                } else {
                    consecutive_failures = 0;
                }
                rt_rw_sem.release();
            }
        } else {
            consecutive_failures = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}


bool collect_cb(quadlet_t* data, short num_avail) {
    for (int i = 0; i < num_avail; i++) {
        quadlet_t value = data[i];
        bool type = ((value&0x80000000)>>31);
        bool timer_overflow = ((value&0x40000000)>>30);
        int timer = ((value&0x3FFF0000)>>16);
        int data = (value&0x0000FFFF);
        if (type == 1) {
            current_history.push_back((data - 0x8000) / amps_to_bits[tuning_axis_index]);
            current_history_t.push_back(1.0f / 80e3 * current_history.size());
            // printf("collect %d \n", num_avail);
        }
        // printf("sz %d \n", current_history.size());
        // printf("type %d, ovf %d, timer %d, data %d\n", type, timer_overflow, timer, data);
    }
    // if (num_avail != 0) printf("dat col cb %d buffer %d\n", num_avail, current_history.size());
    if (current_history.size() > tuning_pulse_width && board->IsCollecting()) {
        board->DataCollectionStop();
        board->SetMotorCurrent(tuning_axis_index, 0x8000);
    }
    // return num_avail != 0 && board->IsCollecting();
    return true;
}


static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

int main(int, char**)
{
    current_history.reserve(65536);
    current_history_t.reserve(65536);  
    
    // Setup window
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100
    const char* glsl_version = "#version 100";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    // GL 3.2 + GLSL 150
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

    // Create window with graphics context
    GLFWwindow* window = glfwCreateWindow(1280, 720, "dRAC factory test", NULL, NULL);
    if (window == NULL)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

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
    // ImGuiIO& io = ImGui::GetIO();
    // io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/noto/NotoMono-Regular.ttf", 16.0f);
    // io.Fonts->AddFontFromFileTTF("FiraMono-Medium.ttf", 16.0f);

    // Our state
    bool show_demo_window = false;
    bool show_another_window = true;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    cell_bg_pass = ImGui::GetColorU32(ImVec4(0.0f, 0.3f, 0.0f, 0.7f));
    cell_bg_fail = ImGui::GetColorU32(ImVec4(0.5f, 0.0f, 0.0f, 0.7f));
    cell_bg_in_progress = ImGui::GetColorU32(ImVec4(0.3f, 0.3f, 0.0f, 0.5f));
    cell_bg_untested = ImGui::GetColorU32(ImVec4(1.0f, 1.0f, 1.0f, 0.1f));
    std::thread(update_all_boards).detach();

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        ImGui::SetNextWindowSize(ImVec2(width, height)); // ensures ImGui fits the GLFW window
        ImGui::SetNextWindowPos(ImVec2(0, 0));

        ImGui::Begin("dRAC factory test");
        ImGui::Text("This program tests the dRAC boards with special termination plug. Do not run the test with robot connected. You may damage the robot.");
        ImGui::PushItemWidth(100);
        // ImGui::InputInt("Command to read delay", &command_wait_time, 10, 100);
        // ImGui::SameLine();
        // ImGui::InputInt("kp", &kp);
        // ImGui::SameLine();
        // ImGui::InputInt("ki", &ki);                 
        static int ethfw = 1;
        ImGui::InputInt("Board number", &board_id);  ImGui::SameLine();
        ImGui::RadioButton("Firewire", &ethfw, 0); ImGui::SameLine();
        ImGui::RadioButton("Ethernet", &ethfw, 1); ImGui::SameLine();

        if (ImGui::Button("Connect", ImVec2(300, 0))) {
            const std::lock_guard<std::mutex> lock(port_mutex);
            board_connected = false;
            BasePort::ProtocolType protocol = BasePort::PROTOCOL_SEQ_RW;
            std::string portDescription = ethfw ? "udp" : "fw";
            std::stringstream().swap(ss_console);
            Port = PortFactory(portDescription.c_str(), ss_console);
	        sleep(500, false);
            if (Port && Port->IsOK()) {
                board = new AmpIO(board_id);
                Port->AddBoard(board);
                board_connected = (Port && Port->GetNumOfNodes() > 0);
                board_connected = (Port && Port->GetNumOfNodes() > 0);
                BoardSN.clear();
                BoardSNRead.clear();

                if (board_connected) {
                    // const std::lock_guard<std::mutex> lock(port_mutex);
                    sleep(10, false);
                    BoardSNRead = board->GetQLASerialNumber(0);
                    BoardSN = BoardSNRead;
                    board->WriteWatchdogPeriodInSeconds(0.1);
                    Port->WriteQuadlet(board_id, 0xB100, 0b11111110); // bypass safety, except the watchdog
                    Port->WriteAllBoards();                        
                    // for (int i = 0; i < num_axes; i++){
                    //     board->WriteCurrentKpRaw(i, kp);
                    //     board->WriteCurrentKiRaw(i, ki);
                    //     board->WriteCurrentITermLimitRaw(i, 100);
                    // }
                }
            }
        }
        ImGui::SameLine();
        if (board_connected) {
            ImGui::TextColored(green, "Connected");
        } else {
            ImGui::TextColored(red, "Not connected");
        }
        // ImGui::SameLine();
       
        ImGui::Separator();


        // EthBasePort *ethPort = dynamic_cast<EthBasePort *>(Port);
        if (board_connected) {
            ImGui::Separator();
            ImGui::BeginTable("rt monitor", 11);
            ImGui::TableNextColumn();
            ImGui::Text("Channel");
            for (int axis_index = 0; axis_index < 10; axis_index++) {
                ImGui::TableNextColumn();
                ImGui::Text(channel_names[axis_index]);
            }
            ImGui::TableNextColumn();
            ImGui::Text("Power");
            for (int axis_index = 0; axis_index < 10; axis_index++) {
                ImGui::TableNextColumn();
                auto axis_en = board->GetAmpEnable(axis_index);
                ImGui::TextColored(axis_en ? green : white, axis_en ? "On" : "Off");
            }
            ImGui::TableNextColumn();
            ImGui::Text("Current");
            for (int axis_index = 0; axis_index < 10; axis_index++) {
                motor_current_read[axis_index] = board->GetMotorCurrent(axis_index);
                ImGui::TableNextColumn();
                // ImGui::Text(channel_names[axis_index]);
                ImGui::Text("0x%04X", motor_current_read[axis_index]);
                ImGui::Text("% 1.03f A", (motor_current_read[axis_index] - 0x8000) / amps_to_bits[axis_index]);
            }
            ImGui::TableNextColumn();
            ImGui::Text("Voltage");
            for (int axis_index = 0; axis_index < 10; axis_index++) {
                motor_voltage_read[axis_index] = board->GetMotorVoltageRatio(axis_index);
                ImGui::TableNextColumn();
                ImGui::Text("% 2.03f V", mv * motor_voltage_read[axis_index]);
            }
            ImGui::TableNextColumn();
            ImGui::Text("Fault");
            ImGui::SameLine();
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Faults will disable the corresponding channel. Faults are sticky and can be cleared when enabling the channel. \nHW overcurrent and overtemp faults will affect the other channel on the same chip.\nIf ADC satuation, check AD4008 and INA240. If HW overcurrent or overtemp, check DRV8432.");

            for (int axis_index = 0; axis_index < 10; axis_index++) {
                amp_fault_codes[axis_index] = board->GetAmpFaultCode(axis_index);
                ImGui::TableNextColumn();
                if (amp_fault_codes[axis_index] == 0) {
                    // ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_pass);
                } else {
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_fail);
                }
                ImGui::Text(amp_fault_text[amp_fault_codes[axis_index]]);
            }
            ImGui::EndTable();
            ImGui::PopItemWidth();
            ImGui::PushItemWidth(200);
            

            ImGui::Separator();

            if (ImGui::Button("board enable")) {
                const std::lock_guard<std::mutex> lock(port_mutex);
                board->SetPowerEnable(true);
            }

            ImGui::InputInt("Tuning axis", &tuning_axis_index);
            ImGui::SameLine();
            ImGui::Text(channel_names[tuning_axis_index]);



            if (ImGui::Button("read")) {
                const std::lock_guard<std::mutex> lock(port_mutex);
                kp = board->ReadCurrentKpRaw(tuning_axis_index);
                ki = board->ReadCurrentKiRaw(tuning_axis_index);
                ilim = board->ReadCurrentITermLimitRaw(tuning_axis_index);
                olim = board->ReadDutyCycleLimit(tuning_axis_index);
            }
            
            if (ImGui::Button("enable")) {
                const std::lock_guard<std::mutex> lock(port_mutex);
                board->SetAmpEnable(tuning_axis_index, true);
            }

            ImGui::InputInt("tuning pulse width", &tuning_pulse_width, 1, 10);
            ImGui::DragInt("kp", &kp, 1, 0, 0x3ffff);
            ImGui::DragInt("ki", &ki, 1, 0, 0x3ffff);
            ImGui::InputInt("i term lim", &ilim, 1, 10);
            ImGui::InputInt("duty cycle lim", &olim, 1, 10);    
            if (ImGui::Button("write")) {
                const std::lock_guard<std::mutex> lock(port_mutex);
                Port->WriteQuadlet(board_id, 0x9000 | (tuning_axis_index+1) << 4 | 0x0c, tuning_pulse_width);
                board->WriteCurrentKpRaw(tuning_axis_index, kp);
                board->WriteCurrentKiRaw(tuning_axis_index, ki);
                board->WriteCurrentITermLimitRaw(tuning_axis_index, ilim);
                board->WriteDutyCycleLimit(tuning_axis_index, olim);

            }
            if (ImGui::Button("pulse 0.1A")) {
                const std::lock_guard<std::mutex> lock(port_mutex);
                current_history.clear();
                current_history_t.clear();
                board->DataCollectionStart(tuning_axis_index + 1, collect_cb);
                board->SetMotorCurrent(tuning_axis_index, 0.1 * amps_to_bits[tuning_axis_index] + 0x8000);
            }
            ImGui::Text(board->IsCollecting() ? "Collecting" : "Not collecting");
            // current_history_t.push_back(0.1);
            // current_history.push_back(0.1);
            static ImPlotFlags flags = ImPlotFlags_NoLegend;
            static ImPlotAxisFlags xflags = ImPlotAxisFlags_AutoFit|ImPlotAxisFlags_NoGridLines;
            static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit|ImPlotAxisFlags_NoGridLines;
            if (ImPlot::BeginPlot("##Tuning","t","I",ImVec2(-1,-1),flags,xflags,yflags)) {
                ImPlot::PlotLine("I measured", current_history_t.data(), current_history.data(), tuning_pulse_width + 200);
                ImPlot::EndPlot();
            }            
        }
        ImGui::Text(ss_console.str().c_str());



        ImGui::End();

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
