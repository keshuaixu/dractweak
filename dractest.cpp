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

const std::string version = "1.0.0";
const auto mv_bit_to_volt = 1.31255e-3;
int command_wait_time = 20;
int kp = 400;
int ki = 20;
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


void read_quadlet_threadsafe(unsigned char board_id, nodeaddr_t address, quadlet_t &data) {
    const std::lock_guard<std::mutex> lock(port_mutex);
    Port->ReadQuadlet(board_id, address, data);
}

void write_quadlet_threadsafe(unsigned char board_id, nodeaddr_t address, quadlet_t data) {
    const std::lock_guard<std::mutex> lock(port_mutex);
    Port->WriteQuadlet(board_id, address, data);
}

enum result_t {
    FAIL = 3,
    PASS = 2,
    IN_PROGRESS = 1,
    UNTESTED = 0
};

result_t result_safety_relay_open = UNTESTED;
result_t result_safety_relay_close = UNTESTED;
result_t result_safety_chain = UNTESTED;
result_t result_48V_on = UNTESTED;
result_t result_48V_off = UNTESTED;
result_t result_LVDS_loopback = UNTESTED;
std::array<result_t, num_axes> result_adc_zero = {UNTESTED};
std::array<result_t, num_axes> result_drive_pos = {UNTESTED};
std::array<result_t, num_axes> result_drive_neg = {UNTESTED};
std::array<result_t, num_axes> result_drive_close_loop_pos = {UNTESTED};
std::array<result_t, num_axes> result_drive_close_loop_neg = {UNTESTED};


std::array<float, num_axes> drive_pos_current = {0.0};
std::array<float, num_axes> drive_neg_current = {0.0};
std::array<float, num_axes> drive_pos_current_closed_loop = {0.0};
std::array<float, num_axes> drive_neg_current_closed_loop = {0.0};
std::array<quadlet_t, num_axes> adc_zero = {0};


void reset_results() {
    result_safety_relay_open = UNTESTED;
    result_safety_relay_close = UNTESTED;
    result_safety_chain = UNTESTED;
    result_48V_on = UNTESTED;
    result_48V_off = UNTESTED;
    result_LVDS_loopback = UNTESTED;
    result_adc_zero.fill(UNTESTED);
    result_drive_pos.fill(UNTESTED);
    result_drive_neg.fill(UNTESTED);
    result_drive_close_loop_pos.fill(UNTESTED);
    result_drive_close_loop_neg.fill(UNTESTED);
    drive_pos_current.fill(0.0);
    drive_neg_current.fill(0.0);
    drive_pos_current_closed_loop.fill(0.0);
    drive_neg_current_closed_loop.fill(0.0);
    adc_zero.fill(0);
}

void color_result(result_t result) {
    switch (result) {
        case PASS:
            ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_pass);
            break;
        case FAIL:
            ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_fail);
            break;
        case IN_PROGRESS:
            ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_in_progress);
            break;
        case UNTESTED:
            ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_untested);
            break;
    }
}

std::string get_result_string(result_t result) {
    switch (result) {
        case PASS:
            return "[Pass]";
        case FAIL:
            return "[Fail]";
        case IN_PROGRESS:
            return "[In Progress]";
        case UNTESTED:
            return "[Untested]";
    }
}

void display_result(result_t r) {
    ImGui::Text(get_result_string(r).c_str());
    color_result(r);
}

void test_safety_relay(){
    {
        const std::lock_guard<std::mutex> lock(port_mutex);
        board->SetSafetyRelay(0);
    }
    sleep(50);
    if (board->GetSafetyRelayStatus() == 0) result_safety_relay_open = PASS; else result_safety_relay_open = FAIL;
    {
        const std::lock_guard<std::mutex> lock(port_mutex);    
        board->SetSafetyRelay(1);
    }
    sleep(50);
    if (board->GetSafetyRelayStatus() == 1) result_safety_relay_close = PASS; else result_safety_relay_close = FAIL;
}

float mv_volts_off;
float mv_volts_on;
void test_48v() {
    quadlet_t mv;
    read_quadlet_threadsafe(board_id, 0xb002, mv);
    mv_volts_on = mv_bit_to_volt * mv;
    if (mv_volts_on > 9.0) result_48V_on = PASS; else result_48V_on = FAIL;
}

void test_safety_chain() {
    quadlet_t mv;
    board->SetSafetyRelay(1);
    board->SetPowerEnable(1);
    sleep(500);
    board->SetSafetyRelay(0);
    sleep(500);
    if (mv_volts_off < 5.0) result_safety_chain = PASS; else result_safety_chain = FAIL;
}

const int ADC_ZERO_TOLERANCE = 0x40;
void test_adc_zero() {
    bool adc_zero_fail[10] = {false};
    for (int i = 0; i < 100; i++) {
        for (int index = 0; index < num_axes; index++) {
            auto current = board->GetMotorCurrent(index);
            if (i == 0) {
                adc_zero[index] = current;
            } else {
                if (current < 0x8000 - ADC_ZERO_TOLERANCE || current > 0x8000 + ADC_ZERO_TOLERANCE) {
                    adc_zero[index] = current;
                    adc_zero_fail[index] = true;
                }
            }
        }
        sleep(1);
    }

    for (int index = 0; index < num_axes; index++) {
        if (adc_zero_fail[index]) {
            result_adc_zero[index] = FAIL;
        } else {
            result_adc_zero[index] = PASS;
        }
    }
}

// const float resistance = 0.5;
const float test_current = 1.0;
const float current_threshold = 0.2;

void test_open_loop_drive() {
    sleep(command_wait_time);
    auto mv = std::max(mv_volts_on, 10.0f); // avoid super high current caused by erronous mv sense
    quadlet_t pos_dir_adc[num_axes] = {0};
    quadlet_t neg_dir_adc[num_axes] = {0};
    for (int index = 0; index < num_axes; index++) {
        {
            const std::lock_guard<std::mutex> lock(port_mutex);
            board->SetAmpEnable(index, 1);
            sleep(command_wait_time, false);
            board->SetMotorVoltageRatio(index, (test_current * internal_resistance[index]) / mv);
        }
        sleep(command_wait_time);
        pos_dir_adc[index] = board->GetMotorCurrent(index);
        auto i_pos = ((signed)pos_dir_adc[index] - 0x8000) / amps_to_bits[index];
        drive_pos_current[index] = i_pos;
        if ( std::abs(i_pos - test_current) < current_threshold ) {
            result_drive_pos[index] = PASS;
        } else {
            result_drive_pos[index] = FAIL;
        }
        {
            const std::lock_guard<std::mutex> lock(port_mutex);
            board->SetMotorVoltageRatio(index, -(test_current * internal_resistance[index]) / mv);
        }
        sleep(command_wait_time);
        neg_dir_adc[index] = board->GetMotorCurrent(index);
        {
            const std::lock_guard<std::mutex> lock(port_mutex);
            board->SetAmpEnable(index, 0);
        }
        auto i_neg = ((signed)neg_dir_adc[index] - 0x8000) / amps_to_bits[index];
        drive_neg_current[index] = i_neg;
        if ( std::abs(i_neg + test_current) < current_threshold ) {
            result_drive_neg[index] = PASS;
        } else {
            result_drive_neg[index] = FAIL;
        }
    }
}

const float current_threshold_closed_loop = 0.05;


void test_closed_loop_drive() {
    sleep(command_wait_time);
    auto mv = std::max(mv_volts_on, 10.0f); // avoid super high current caused by erronous mv sense
    quadlet_t pos_dir_adc[num_axes] = {0};
    quadlet_t neg_dir_adc[num_axes] = {0};
    for (int index = 0; index < num_axes; index++) {
        {
            const std::lock_guard<std::mutex> lock(port_mutex);
            sleep(command_wait_time, false);
            board->SetAmpEnable(index, 1);
        }
        sleep(command_wait_time);
        {
            const std::lock_guard<std::mutex> lock(port_mutex);
            board->SetMotorCurrent(index, (test_current * amps_to_bits[index]) + 0x8000);
        }
        sleep(command_wait_time);
        pos_dir_adc[index] = board->GetMotorCurrent(index);
        auto i_pos = ((signed)pos_dir_adc[index] - 0x8000) / amps_to_bits[index];
        drive_pos_current_closed_loop[index] = i_pos;
        if ( std::abs(i_pos - test_current) < current_threshold_closed_loop ) {
            result_drive_close_loop_pos[index] = PASS;
        } else {
            result_drive_close_loop_pos[index] = FAIL;
        }
        {
            const std::lock_guard<std::mutex> lock(port_mutex);
            board->SetMotorCurrent(index, (-test_current * amps_to_bits[index]) + 0x8000);
        }
        sleep(command_wait_time);
        neg_dir_adc[index] = board->GetMotorCurrent(index);
        {
            const std::lock_guard<std::mutex> lock(port_mutex);
            board->SetAmpEnable(index, 0);
        }
        auto i_neg = ((signed)neg_dir_adc[index] - 0x8000) / amps_to_bits[index];
        drive_neg_current_closed_loop[index] = i_neg;
        if ( std::abs(i_neg + test_current) < current_threshold_closed_loop ) {
            result_drive_close_loop_neg[index] = PASS;
        } else {
            result_drive_close_loop_neg[index] = FAIL;
        }
    }
}

void test_all() {
    testing = true;

    {
        const std::lock_guard<std::mutex> lock(port_mutex);
        Port->WriteQuadlet(board_id, 0xB100, 0b11111110); // bypass safety, except the watchdog
        for (int i = 0; i < num_axes; i++) {
            board->WriteCurrentKpRaw(i, kp);
            board->WriteCurrentKiRaw(i, ki);
            board->WriteCurrentITermLimitRaw(i, 100);
        }
    }
    test_safety_relay();
    {
        const std::lock_guard<std::mutex> lock(port_mutex);
        board->SetPowerEnable(1);
    }
    sleep(1000);
    test_48v();
    // test_safety_chain();
    test_adc_zero();
    test_open_loop_drive();
    test_closed_loop_drive();
    {
        const std::lock_guard<std::mutex> lock(port_mutex);
        board->SetPowerEnable(0);
    }

    // test_lvds_loopback();
    std::stringstream filename;
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::filesystem::create_directory("dRAC_test_results");
    filename << "dRAC_test_results/dRAC_" << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S") << "_" << BoardSNRead << ".txt";
    std::ofstream logfile;
    logfile.open(filename.str(), std::fstream::out);
    logfile << "dRAC Factory Test Log" << std::endl;
    logfile << "Version: " << version << std::endl;
    logfile << "Board SN: " << BoardSNRead << std::endl;
    logfile << "Local time: " << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << std::endl;
    logfile << "Safety relay open: " << get_result_string(result_safety_relay_open) << std::endl;
    logfile << "Safety relay close: " << get_result_string(result_safety_relay_open) << std::endl;
    logfile << "MV sense: " << mv_volts_on << std::endl;

    logfile << "ADC zero:" << " ";
    for (int i = 0; i < num_axes; i++) {
        logfile << channel_names[i] << "=" << get_result_string(result_adc_zero[i]) << " ";
    }
    logfile << std::endl;

    logfile << "Amps CV+:" << " ";
    for (int i = 0; i < num_axes; i++) {
        logfile << channel_names[i] << "=" << drive_pos_current[i] << " ";
    }
    logfile << std::endl;

    logfile << "Amps CV-:" << " ";
    for (int i = 0; i < num_axes; i++) {
        logfile << channel_names[i] << "=" << drive_neg_current[i] << " ";
    }
    logfile << std::endl;

    logfile << "Amps CC+:" << " ";
    for (int i = 0; i < num_axes; i++) {
        logfile << channel_names[i] << "=" << drive_pos_current_closed_loop[i] << " ";
    }
    logfile << std::endl;

    logfile << "Amps CC-:" << " ";
    for (int i = 0; i < num_axes; i++) {
        logfile << channel_names[i] << "=" << drive_neg_current_closed_loop[i] << " ";
    }
    logfile << std::endl;
    logfile << std::endl;


    logfile << "Read buffer: " << std::endl;
    board->DisplayReadBuffer(logfile);

    logfile << std::endl;
    logfile.close();

    testing = false;
}

void update_all_boards() {
    static int consecutive_failures = 0;
    while (true) {
        if (board_connected){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            const std::lock_guard<std::mutex> lock(port_mutex);
            if (Port && Port->IsOK()) {
                auto w_result = Port->WriteAllBoards();
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                auto r_result = Port->ReadAllBoards();
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
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


static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

int main(int, char**)
{
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
        ImGui::InputInt("Command to read delay", &command_wait_time, 10, 100);
        ImGui::SameLine();
        ImGui::InputInt("kp", &kp);
        ImGui::SameLine();
        ImGui::InputInt("ki", &ki);                 
        static int ethfw = 1;
        ImGui::InputInt("Board number", &board_id);  ImGui::SameLine();
        ImGui::RadioButton("Firewire", &ethfw, 0); ImGui::SameLine();
        ImGui::RadioButton("Ethernet", &ethfw, 1); ImGui::SameLine();

        if (ImGui::Button("Connect", ImVec2(300, 0))) {
            const std::lock_guard<std::mutex> lock(port_mutex);
            board_connected = false;
            reset_results();
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
                    for (int i = 0; i < num_axes; i++){
                        board->WriteCurrentKpRaw(i, kp);
                        board->WriteCurrentKiRaw(i, ki);
                        board->WriteCurrentITermLimitRaw(i, 100);
                    }
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

            ImGui::InputText("SN to program", &BoardSN); ImGui::SameLine();
            if (ImGui::Button("Program dRAC SN", ImVec2(300, 0))) {
                const std::lock_guard<std::mutex> lock(port_mutex);

                std::string BoardType = "dRA";
                uint8_t wbyte;
                uint16_t address;
                std::stringstream ss;

                ss << BoardType << " " << BoardSN;
                auto str = ss.str();

                // S1: program to QLA PROM
                address = 0x0000;
                for (size_t i = 0; i < str.length(); i++) {
                    wbyte = str.at(i);
                    if (!board->PromWriteByte25AA128(address, wbyte, 0)) {
                        std::cerr << "Failed to write byte " << i << std::endl;
                    }
                    address += 1;  // inc to next byte
                }
                // Terminating byte can be 0 or 0xff
                wbyte = 0;
                if (!board->PromWriteByte25AA128(address, wbyte, 0)) {
                    std::cerr << "Failed to write terminating byte" << std::endl;
                }

                // S2: read back and verify
                BoardSNRead.clear();
                BoardSNRead = board->GetQLASerialNumber(0);
                std::cout << "Read SN = " << BoardSNRead << std::endl;

                if (BoardSN == BoardSNRead) {
                    std::cout << "Programmed dRA " << BoardSN << " Serial Number" << std::endl;
                } else {
                    std::cerr << "Failed to program" << std::endl;
                    std::cerr << "Board SN = " << BoardSN << "\n"
                            << "Read  SN = " << BoardSNRead << std::endl;
                }

            }
            ImGui::SameLine();

            if (BoardSNRead.empty()){
                ImGui::TextColored(red, "Read SN is empty");
            } else if (BoardSN == BoardSNRead) {
                ImGui::TextColored(green, "Read SN = %s", BoardSNRead.c_str());
            } else {
                ImGui::TextColored(red, "Read SN = %s", BoardSNRead.c_str());
            }
            ImGui::SameLine();
            ImGui::Dummy(ImVec2(50.0f, 0.0f));
            ImGui::SameLine();

            if (testing) ImGui::BeginDisabled();
            if (ImGui::Button("Run factory test", ImVec2(300, 0))) {
                reset_results();
                std::thread(test_all).detach();
            }
            if (testing) {
                ImGui::EndDisabled();
                ImGui::SameLine();
                ImGui::TextColored(green, "Testing...");
            }
            // if (ImGui::Button("Test thread")) {
            //     // board
            //     const std::lock_guard<std::mutex> lock(port_mutex);
            //     bool write_state = 0;
                
            //     board->SetPowerEnable(1);
            //     // sleep(10);
            //     int error_count = 0;
            //     for (int i = 0; i< 50; i++) {
            //         {
            //             // const std::lock_guard<std::mutex> lock(port_mutex);
            //             board->SetAmpEnable(0, write_state);
            //         }
            //         Port->WriteAllBoards();
            //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
            //         Port->ReadAllBoards();
            //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
            //         // sleep(10);

            //         auto read_state = board->GetAmpEnable(0);
            //         if (read_state != write_state) error_count ++;
            //         write_state ^= 1;
            //     }
            //     board->SetPowerEnable(0);
            //     std::cout << "err count " << error_count << std::endl; 
            // }


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

            ImGui::Separator();
            ImGui::BeginTable("board monitor", 11);
            ImGui::TableNextColumn();
            ImGui::Text("Status");
            ImGui::TableNextColumn();
            ImGui::Text("LVDS loopback");
            bool lvds_loopback_pass = board->GetStatus() & 0x1;
            if (lvds_loopback_pass) {
                ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_pass);
                ImGui::Text("[OK]");
            } else {
                ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_fail);
                ImGui::Text("[Fail]");
            }
            ImGui::SameLine();
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Check LVDS transceivers U9 and U10");
            ImGui::TableNextColumn();
            ImGui::Text("6V");
            if (board->GetStatus() & (0x1 << 2)) {
                ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_pass);
                ImGui::Text("[OK]");
            } else {
                ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_fail);
                ImGui::Text("[Fail]");
            }
            ImGui::SameLine();
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Check 6V DC/DC U12");
            ImGui::EndTable();

            ImGui::Separator();

            ImGui::BeginTable("tests", 11);
            ImGui::TableNextColumn();
            ImGui::Text("Board");
            ImGui::TableNextColumn();
            ImGui::Text("Relay open");
            display_result(result_safety_relay_open);
            ImGui::SameLine();
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Check relay and Q3.");            
            ImGui::TableNextColumn();
            ImGui::Text("Relay close");
            display_result(result_safety_relay_close);
            ImGui::SameLine();
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Check relay and Q3.");               

            // ImGui::TableNextRow();
            // ImGui::TableNextColumn();
            // ImGui::Text("48V");
            // ImGui::TableNextColumn();
            // ImGui::Text("Off");
            // ImGui::Text("%.2f V", mv_volts_off);
            // display_result(result_48V_off);
            ImGui::TableNextColumn();
            ImGui::Text("MV sense");
            ImGui::Text("%.2f V", mv_volts_on);
            display_result(result_48V_on);
            ImGui::SameLine();
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Should be motor power supply voltage (12V or 48V). Check U1.");

            // ImGui::TableNextRow();
            // ImGui::TableNextColumn();
            // ImGui::Text("Safety chain");
            // ImGui::TableNextColumn();
            // ImGui::Text("Cut");
            // display_result(result_safety_chain);

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("ADC zero");
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("ADC reading should be zero when the amps are disabled. If not, check AD4008 and INA240.");

            for (int i = 0; i < num_axes; i++) {
                ImGui::TableNextColumn();
                // ImGui::Text(channel_names[i]);
                ImGui::Text("0x%04X", adc_zero[i]);
                display_result(result_adc_zero[i]);
            }

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("Amps CV+");
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Drives the output in a constant PWM duty cycle to deliver approx 1A into the internal resistance of 0.5 Ohm.\nIf fail, check DRV8432 and the high current path.");

            for (int i = 0; i < num_axes; i++) {
                ImGui::TableNextColumn();
                // ImGui::Text(channel_names[i]);
                ImGui::Text("%.3f A", drive_pos_current[i]);
                display_result(result_drive_pos[i]);
            }

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("Amps CV-");
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Drives the output in a constant PWM duty cycle to deliver approx -1A into the internal resistance of 0.5 Ohm.\nIf fail, check DRV8432 and the high current path.");

            for (int i = 0; i < num_axes; i++) {
                ImGui::TableNextColumn();
                // ImGui::Text(channel_names[i]);
                ImGui::Text("%.3f A", drive_neg_current[i]);
                display_result(result_drive_neg[i]);
            }

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("Amps CC+");
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Drives the output in current loop to deliver 1A. Many things can cause failure. Use the result from other tests to narrow down the problem.");

            for (int i = 0; i < num_axes; i++) {
                ImGui::TableNextColumn();
                // ImGui::Text(channel_names[i]);
                ImGui::Text("%.3f A", drive_pos_current_closed_loop[i]);
                display_result(result_drive_close_loop_pos[i]);
            }

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("Amps CC-");
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Drives the output in current loop to deliver -1A. Many things can cause failure. Use the result from other tests to narrow down the problem.");

            for (int i = 0; i < num_axes; i++) {
                ImGui::TableNextColumn();
                // ImGui::Text(channel_names[i]);
                ImGui::Text("%.3f A", drive_neg_current_closed_loop[i]);
                display_result(result_drive_close_loop_neg[i]);
            }


            ImGui::EndTable();
            ImGui::Separator();


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
