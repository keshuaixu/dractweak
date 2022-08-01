#define IMGUI_IMPL_OPENGL_LOADER_GLEW


#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>


#include <stdlib.h>
#include <vector>
#include <deque>
#include <algorithm>

#include <Amp1394/AmpIORevision.h>
#include "PortFactory.h"
#include "AmpIO.h"
#include "Amp1394Time.h"
#include "Amp1394BSwap.h"


#include <thread>
#include "EthBasePort.h"
#include <mutex> 

#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
// About Desktop OpenGL function loaders:
//  Modern desktop OpenGL doesn't have a standard portable header file to load OpenGL function pointers.
//  Helper libraries are often used for this purpose! Here we are supporting a few common ones (gl3w, glew, glad).
//  You may use another loader/header of your choice (glext, glLoadGen, etc.), or chose to manually implement your own.
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
#include <GL/gl3w.h>            // Initialize with gl3wInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
#include <GL/glew.h>            // Initialize with glewInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
#include <glad/glad.h>          // Initialize with gladLoadGL()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD2)
#include <glad/gl.h>            // Initialize with gladLoadGL(...) or gladLoaderLoadGL()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING2)
#define GLFW_INCLUDE_NONE       // GLFW including OpenGL headers causes ambiguity or multiple definition errors.
#include <glbinding/Binding.h>  // Initialize with glbinding::Binding::initialize()
#include <glbinding/gl/gl.h>
using namespace gl;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING3)
#define GLFW_INCLUDE_NONE       // GLFW including OpenGL headers causes ambiguity or multiple definition errors.
#include <glbinding/glbinding.h>// Initialize with glbinding::initialize()
#include <glbinding/gl/gl.h>
using namespace gl;
#else
#include IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#endif

// Include glfw3.h after our OpenGL definitions
#include <GLFW/glfw3.h>

#include "implot.h"


static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

float current_from_adc_count(int count, int index) {
    float scale = index < 2 ? 4800.0 : 16000.0;
    return (count - 0x8000) / scale;
}


int dac_count_from_current(float current, int index) {
    float scale = index < 2 ? 4800.0 : 16000.0;
    int result = int(current * scale) + 0x8000;
    return std::clamp(result, 0, 0xffff);
}

float Imax(int index) {
    return current_from_adc_count(0xffff, index);
}

// uint16_t pi_fixed_from_float(float val) {
//     int result = int(val * 4096);
//     return std::clamp(result, 0, 0xffff);
// }

const char* current_format = "% .03f A";
const char* voltage_format = "% .03f V";
const float current_max = 4.5; 

uint16_t debug_write_count = 0;

AmpIO *board;
BasePort *Port;
int BoardId = 0;


int axis = 1;
float i_setp;
int mode;

int kp, ki;
int ilim, olim;

// bool tuning_mode = false;
int tuning_pulse_width = 100;
std::vector<float> current_history;
// std::vector<float> command_history;
float command_history[4] = {0};
float command_history_t[4] = {0};

std::vector<float> current_history_t;
int tuning_axis_index;

float plot_y[1000];
int plot_y_index = 0;


GLFWwindow* window;

uint16_t render_counter = 0;

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
    float im = current_from_adc_count(current, axis - 1);
    ImGui::LabelText("I meas", current_format, im);
    ImGui::Separator();
    ImGui::LabelText("duty cycle", "%d", board->ReadDutyCycle(axis - 1));
    uint32_t fault = board->ReadFault(axis - 1);
    ImGui::LabelText("fault", "%X", fault);
    quadlet_t debug;
    quadlet_t debug2;
    Port->ReadQuadlet(BoardId, (axis) << 4 | 0x0f | 0x9000, debug);
    Port->ReadQuadlet(BoardId, (axis) << 4 | 0x0e | 0x9000, debug2);
    ImGui::LabelText("debug", "0x%08X", debug);
    ImGui::LabelText("debug2", "0x%08X", debug2);
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
    i_setp = current_from_adc_count(i_setp_q, axis - 1);
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
    i_setp_q = dac_count_from_current(i_setp, axis - 1);

    if (setp_edited) {
        Port->WriteQuadlet(BoardId, (axis) << 4 | 0x01, i_setp_q);
    }
    ImGui::Separator();

    int mode_read = board->ReadMotorControlMode(axis - 1);
    if (ImGui::RadioButton("current", mode_read == AmpIO::MotorControlMode::CURRENT)) {
        if (ImGui::IsItemEdited()) {
            board->WriteMotorControlMode(axis - 1, AmpIO::MotorControlMode::CURRENT);
        }
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("voltage", mode_read == AmpIO::MotorControlMode::VOLTAGE)) {
        if (ImGui::IsItemEdited()) {
            board->WriteMotorControlMode(axis - 1, AmpIO::MotorControlMode::VOLTAGE);
        }
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("reset", mode_read == AmpIO::MotorControlMode::RESET)) {
        if (ImGui::IsItemEdited()) {
            board->WriteMotorControlMode(axis - 1, AmpIO::MotorControlMode::RESET);
        }
    }

    if (true) {
        uint32_t stuff = board->ReadCurrentKpRaw(axis - 1);
        kp = board->ReadCurrentKpRaw(axis - 1);
        ki = board->ReadCurrentKiRaw(axis - 1);
        ilim = board->ReadCurrentITermLimitRaw(axis - 1);
        olim = board->ReadDutyCycleLimit(axis - 1);
    }
    if (ImGui::Button("load default")) {
        kp = 1;
        ki = 1;
        ilim = 200;
        olim = 1020;
        board->WriteCurrentKpRaw(axis - 1, kp);
        board->WriteCurrentKiRaw(axis - 1, ki);
        board->WriteCurrentITermLimitRaw(axis - 1, ilim);
        board->WriteDutyCycleLimit(axis - 1, olim);
    }
    ImGui::DragInt("kp", &kp, 1, 0, 0x3ffff);
    if (ImGui::IsItemEdited()) {
        board->WriteCurrentKpRaw(axis - 1, kp);
    }
    ImGui::DragInt("ki", &ki, 1, 0, 0x3ffff);
    if (ImGui::IsItemEdited()) {
        board->WriteCurrentKiRaw(axis - 1, ki);
    }
    ImGui::InputInt("i term lim", &ilim, 1, 10);
    if (ImGui::IsItemEdited()) {
        board->WriteCurrentITermLimitRaw(axis - 1, ilim);
    }
    ImGui::InputInt("duty cycle lim", &olim, 1, 10);    
        if (ImGui::IsItemEdited()) {
        board->WriteDutyCycleLimit(axis - 1, olim);
    }
    

    // ImGui::Checkbox("tuning mode", &tuning_mode);
    ImGui::InputInt("tuning pulse width", &tuning_pulse_width, 1, 10);
    if (ImGui::Button("write")) {
        Port->WriteQuadlet(BoardId, 0x9000 | (axis) << 4 | 0x0c, tuning_pulse_width);
    }
    ImGui::End();
}

bool collect_cb(quadlet_t* data, short num_avail) {
    for (int i = 0; i < num_avail; i++) {
        quadlet_t value = data[i];
        bool type = ((value&0x80000000)>>31);
        bool timer_overflow = ((value&0x40000000)>>30);
        int timer = ((value&0x3FFF0000)>>16);
        int data = (value&0x0000FFFF);
        if (type == 1) {
            current_history.push_back(current_from_adc_count(data, tuning_axis_index));
            current_history_t.push_back(1.0f / 80e3 * current_history.size());
        }
        // printf("sz %d \n", current_history.size());
        // printf("type %d, ovf %d, timer %d, data %d\n", type, timer_overflow, timer, data);
    }
    // if (num_avail != 0) printf("dat col cb %d buffer %d\n", num_avail, current_history.size());
    if (current_history.size() > tuning_pulse_width +500 && board->IsCollecting()) {
        board->DataCollectionStop();
        board->SetMotorCurrent(tuning_axis_index, 0x8000);
    }
    return num_avail != 0;
}


int main(int argc, char** argv)
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
    window = glfwCreateWindow(1440, 720, "dRAC tweak", NULL, NULL);
    if (window == NULL)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(0); // Enable vsync
    bool err = glewInit() != 0;
    if (err)
    {
        fprintf(stderr, "Failed to initialize OpenGL loader!\n");
        return 1;
    }

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
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

    current_history.reserve(65536);
    current_history_t.reserve(65536);    


    BasePort::ProtocolType protocol = BasePort::PROTOCOL_SEQ_RW;

    bool fullvel = false;  // whether to display full velocity feedback
    bool showTime = false; // whether to display time information

    // measure time between reads
    AmpIO_UInt32 maxTime = 0;
    AmpIO_UInt32 lastTime = 0;

    unsigned int curAxis = 0;     // Current axis (0=all, 1-8)
    unsigned int curBoardIndex = 0;
    unsigned int curAxisIndex = 0;
    char axisString[4] = "all";
    std::string portDescription;

    for (int i = 1; i < (unsigned int)argc; i++) {
        if (argv[i][0] == '-') {
            if (argv[i][1] == 'p') {
                portDescription = argv[i]+2;
            }
            else if (argv[i][1] == 'b') {
                // -br -- enable broadcast read/write
                // -bw -- enable broadcast write (sequential read)
                if (argv[i][2] == 'r')
                    protocol = BasePort::PROTOCOL_BC_QRW;
                else if (argv[i][2] == 'w')
                    protocol = BasePort::PROTOCOL_SEQ_R_BC_W;
            }
            else if (argv[i][1] == 'v')
                fullvel = true;
            else if (argv[i][1] == 't')
                showTime = true;
        }
        else {
            int bnum = atoi(argv[i]);
            if ((bnum >= 0) && (bnum < BoardIO::MAX_BOARDS)) {
                board = new AmpIO(bnum);
                BoardId = bnum;
                std::cerr << "Selecting board " << bnum << std::endl;
            }
            else
                std::cerr << "Invalid board number: " << argv[i] << std::endl;
        }
    }

    Port = PortFactory(portDescription.c_str(), std::cout);
    if (!Port) {
        std::cerr << "Failed to create port using: " << portDescription << std::endl;
        return -1;
    }
    if (!Port->IsOK()) {
        std::cerr << "Failed to initialize " << Port->GetPortTypeString() << std::endl;
        return -1;
    }

    EthBasePort *ethPort = dynamic_cast<EthBasePort *>(Port);

    Port->AddBoard(board);

    int num_axes = 10;
    int motor_current_read[num_axes];
    float motor_voltage_read[num_axes];
    float current_setpoint[num_axes] = {0.0};
    float voltage_setpoint[num_axes] = {0.0};

    board->WriteWatchdogPeriodInSeconds(1);

    // std::thread t1([](){
    //     while (!glfwWindowShouldClose(window)) {
    //         mtx.lock();
    //         Port->ReadAllBoards();
    //         mtx.unlock();
    //         // printf("blah\n");
    //         Amp1394_Sleep(0.000001);
    //     }
    // });
    int adc_phase = 0;
    bool espmv = 1;



    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        render_counter++;
        if (! (render_counter % 2)) {
            Port->ReadAllBoards();
            Port->WriteAllBoards();
        } else {
        Port->ReadAllBoards();

        // mtx.lock();
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        glfwPollEvents();
        // Amp1394_Sleep(0.1);

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
            ImGui::BeginTable("boardtable", 3);
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
            bool relay_enable = board->GetSafetyRelayStatus();
            if (ImGui::Checkbox("Relay", &relay_enable)) {
                if (ImGui::IsItemEdited()) {
                    board->SetSafetyRelay(relay_enable);
                }
            }
            if (ImGui::Checkbox("ESPMV", &espmv)) {
                if (ImGui::IsItemEdited()) {
                    Port->WriteQuadlet(BoardId, 0x9003, espmv);
                }
            }

            // ImGui::LabelText("48V voltage", "%.2f", 0);
            // ImGui::Text("motor power = %s", board->GetPowerEnable() ? "on" : "off");
            // ImGui::TableNextColumn();
            if (ImGui::Button("bypass safety")) {
                Port->WriteQuadlet(BoardId, 0xB100, 1);
            }
            ImGui::SameLine();
            if (ImGui::Button("unbypass")) {
                Port->WriteQuadlet(BoardId, 0xB100, 0);
            }  

            if (ImGui::Button("release suj")) {
                Port->WriteQuadlet(BoardId, 0x6, 0x0101);
            }
            ImGui::SameLine();
            if (ImGui::Button("unrelease")) {
                Port->WriteQuadlet(BoardId, 0x6, 0x0100);
            }  


            if (ImGui::Button("send test lvds")) {
                Port->WriteQuadlet(BoardId,0xB101 , 0x0);
                Port->WriteQuadlet(BoardId, 0xB101, 0xAC450F28); 
                for (int i=0; i < 48; i++)
                    Port->WriteQuadlet(BoardId,0xB101 , 0x12121212);
                Port->WriteQuadlet(BoardId,0xB101 , 0x2f1d); 
                // quadlet_t cg;
                // Port->ReadQuadlet(BoardId, 0xb001, cg);            
            }
            ImGui::TableNextColumn();
            // if (ImGui::Button("start")) {
            //     current_history.clear();
            //     int result = board->DataCollectionStart(1, collect_cb);
            //     board->SetMotorCurrent(0, 0x8200);
            //     // std::cout << "start " << result << std::endl;
            // // Amp1394_Sleep(0.100);
            // // board->SetMotorCurrent(axis_index, 0x8000);
            // // board->DataCollectionStop();
            // // Port->WriteAllBoards();W
            // }

            // if (ImGui::Button("dat col stop")) {
            //     board->DataCollectionStop();
            // // board->SetMotorCurrent(0, 0x8000);
            // }
            ImGui::RadioButton("collecting", board->IsCollecting());
            ImGui::RadioButton("watchdog timeout", board->GetWatchdogTimeoutStatus());
            ImGui::RadioButton("mv good", board->GetPowerStatus());

            uint32_t digin = board->GetDigitalInput();
            char name[32];
            for (int i = 0; i < 32; i++) {
                sprintf(name, "##digin%d", i);
                ImGui::RadioButton(name, (digin >> i) & 1);
                if (i % 8 != 7) ImGui::SameLine();
            }

            ImGui::TableNextColumn();
            if (ImGui::Button("default pi")) {
                for (int axis = 1; axis < 11; axis ++) {
                    board->WriteCurrentKpRaw(axis - 1, 0);
                    board->WriteCurrentKiRaw(axis - 1, 10);
                    board->WriteCurrentITermLimitRaw(axis - 1, 200);
                    board->WriteDutyCycleLimit(axis - 1, 1020);
                }
            }
            // ImGui::InputInt("RtLen", &board->rtlen_debug, 1, 1);
            ImGui::DragInt("adc phase", &adc_phase, 1.0f, 0, 2047);
            if (ImGui::IsItemEdited()) {
                Port->WriteQuadlet(BoardId, 0x9002, adc_phase);
            }
            quadlet_t crc_good_count;
            quadlet_t crc_err_count;
            quadlet_t mv;
            quadlet_t drac_status;
            quadlet_t esii_status;
            quadlet_t espm_adc;
            quadlet_t instrument_id;
            quadlet_t instrument_id2;
            quadlet_t dev_build_number;
            uint32_t inst_model = board->SPSMReadToolModel();
            uint8_t inst_version = board->SPSMReadToolVersion();
            Port->ReadQuadlet(BoardId, 0xb000, crc_err_count);
            Port->ReadQuadlet(BoardId, 0xb001, crc_good_count);
            Port->ReadQuadlet(BoardId, 0xb002, mv);
            Port->ReadQuadlet(BoardId, 0xb003, drac_status);
            Port->ReadQuadlet(BoardId, 0xb010, esii_status);
            Port->ReadQuadlet(BoardId, 0xb011, espm_adc);
            Port->ReadQuadlet(BoardId, 0xb012, instrument_id);
            Port->ReadQuadlet(BoardId, 0xb013, instrument_id2);
            Port->ReadQuadlet(BoardId, 0xbfff, dev_build_number);
            ImGui::LabelText("crc good", "%d", crc_good_count);
            ImGui::LabelText("crc err", "%d", crc_err_count);
            ImGui::LabelText("mv", "%.2f V", mv * 1.31255e-3);
            ImGui::LabelText("esii status", "0x%08X", esii_status);
            ImGui::LabelText("espm adc", "0x%08X", espm_adc);
            ImGui::LabelText("instrument1", "0x%08X", instrument_id);
            ImGui::LabelText("instrument2", "0x%08X", instrument_id2);
            ImGui::LabelText("build", "0x%08X", dev_build_number);
            ImGui::LabelText("inst model", "%lu", inst_model);
            ImGui::LabelText("inst v", "%lu", inst_version);
            ImGui::RadioButton("espmv good", (drac_status >> 0) & 1);
            ImGui::RadioButton("safety chain read", (drac_status >> 1) & 1);
            ImGui::RadioButton("espm comm good", (drac_status >> 2) & 1);
            ImGui::RadioButton("esii/cc comm good", (drac_status >> 3) & 1);
            ImGui::RadioButton("is ecm", (drac_status >> 4) & 1);

            // quadlet_t esii_buffer[64];
            // Port->ReadBlock(BoardId, 0xa000, esii_buffer, 64*4);
            // for (int offset = 48; offset < 55; offset += 1) {
            //     ImGui::Text("0x%04X 0x%08X", offset, bswap_32(esii_buffer[offset]));
            // }



            // ImGui::TableNextColumn();
            // ImGui::PlotLines("I", current_history.data(), tuning_pulse_width + 200, 0, NULL, FLT_MAX, FLT_MAX, ImVec2(600, 200), sizeof(float));
            // ImGui::SameLine();
            ImGui::EndTable();

            ImGui::TreePop();

        }
        for (int axis_index = 0; axis_index < num_axes; axis_index++) {
            char window_name[32];
            sprintf(window_name, "Axis %d", axis_index + 1);
            // ImGui::BeginChild(window_name, ImVec2(200, 0), true, ImGuiWindowFlags_None);
            if (ImGui::TreeNodeEx(window_name, ImGuiTreeNodeFlags_DefaultOpen)){
                ImGui::BeginTable("ax", 3);
                ImGui::TableNextColumn();
                bool axis_enable = board->GetAmpEnable(axis_index);
                // printf("axis %d en %d\n", axis_index, axis_enable);
                if (ImGui::Checkbox("Enable", &axis_enable)) {
                    if (ImGui::IsItemEdited()) {
                        board->SetAmpEnable(axis_index, axis_enable);
                    }
                }
                ImGui::LabelText("encoder pos", "0x%08X", board->GetEncoderPosition(axis_index));
                ImGui::ProgressBar((board->GetEncoderPosition(axis_index) & 32767) / 32767.0f,ImVec2(0.0f, 0.0f),"");
                ImGui::LabelText("pot", "0x%04X", board->GetAnalogInput(axis_index));
                ImGui::ProgressBar((board->GetAnalogInput(axis_index) & 4095) / 4095.0f,ImVec2(0.0f, 0.0f),"");
                ImGui::LabelText("fault", "0x%04X", board->GetAmpFaultCode(axis_index));
                ImGui::TableNextColumn();

                ImGui::LabelText("vel", "0x%08X", board->GetEncoderVelocityRaw(axis_index));
                ImGui::LabelText("qtr1", "0x%08X", board->GetEncoderQtr1Raw(axis_index));
                ImGui::LabelText("qtr5", "0x%08X", board->GetEncoderQtr5Raw(axis_index));
                ImGui::LabelText("run", "0x%08X", board->GetEncoderRunningCounterRaw(axis_index));
                ImGui::LabelText("vel predicted", "%f", board->GetEncoderVelocityPredicted(axis_index, 0));
                ImGui::LabelText("vel", "%f", board->GetEncoderVelocity(axis_index));
                ImGui::LabelText("acc", "%f", board->GetEncoderAcceleration(axis_index, 0));
                ImGui::TableNextColumn();


                if (axis_index == axis - 1) {
                    if (++ plot_y_index == 1000) plot_y_index = 0;
                    plot_y[plot_y_index] = (float) (board->GetEncoderPosition(axis_index));
                }

         


                bool need_to_set_current = 0;
                motor_current_read[axis_index] = board->GetMotorCurrent(axis_index);
                ImGui::LabelText("adc", "0x%04X", motor_current_read[axis_index]);
                float im = current_from_adc_count(motor_current_read[axis_index], axis_index);
                ImGui::LabelText("I", current_format, im);
                ImGui::ProgressBar(std::abs(im)/Imax(axis_index),ImVec2(0.0f, 0.0f),"");
                ImGui::DragFloat("I setp", &(current_setpoint[axis_index]), 0.001, -current_max, current_max, current_format);
                need_to_set_current |= ImGui::IsItemEdited();
                if (ImGui::Button("0")) {
                    current_setpoint[axis_index] = 0;
                    need_to_set_current |= 1;
                }
                ImGui::SameLine();
                if (ImGui::Button("0.05")) {
                    current_setpoint[axis_index] = 0.05;
                    need_to_set_current |= 1;
                }
                ImGui::SameLine();           
                if (ImGui::Button("0.5")) {
                    current_setpoint[axis_index] = 0.5;
                    need_to_set_current |= 1;
                }
                ImGui::SameLine();          
                if (ImGui::Button("-0.5")) {
                    current_setpoint[axis_index] = -0.5;
                    need_to_set_current |= 1;
                }

                auto i_setp_q = dac_count_from_current(current_setpoint[axis_index], axis_index);
                if (need_to_set_current) {
                    board->SetMotorCurrent(axis_index, i_setp_q);
                }

                if (ImGui::Button("pulse 0.1A")) {
                    current_history.clear();
                    current_history_t.clear();
                    board->DataCollectionStart(axis_index + 1, collect_cb);
                    board->SetMotorCurrent(axis_index, dac_count_from_current(0.1, axis_index));
                    tuning_axis_index = axis_index;
                }
                ImGui::SameLine();
                if (ImGui::Button("0.5A")) {
                    current_history.clear();
                    current_history_t.clear();
                    board->DataCollectionStart(axis_index + 1, collect_cb);
                    board->SetMotorCurrent(axis_index, dac_count_from_current(0.5, axis_index));
                    tuning_axis_index = axis_index;
                }
                ImGui::SameLine();
                if (ImGui::Button("2A")) {
                    current_history.clear();
                    current_history_t.clear();
                    board->DataCollectionStart(axis_index + 1, collect_cb);
                    board->SetMotorCurrent(axis_index, dac_count_from_current(2.0, axis_index));
                    tuning_axis_index = axis_index;
                }
                ImGui::SameLine();
                if (ImGui::Button("setp")) {
                    current_history.clear();
                    current_history_t.clear();
                    board->DataCollectionStart(axis_index + 1, collect_cb);
                    board->SetMotorCurrent(axis_index, dac_count_from_current(current_setpoint[axis_index], axis_index));
                    tuning_axis_index = axis_index;
                }       

                motor_voltage_read[axis_index] = board->GetMotorVoltageRatio(axis_index);
                float vm = motor_voltage_read[axis_index] * 48.0f;
                ImGui::LabelText("V", voltage_format, vm);
                ImGui::ProgressBar(std::abs(vm)/48.0f,ImVec2(0.0f, 0.0f),"");

                ImGui::DragFloat("V setp", &(voltage_setpoint[axis_index]), 0.01, -48, 48);
                if (ImGui::IsItemEdited()) {
                    board->SetMotorVoltageRatio(axis_index, voltage_setpoint[axis_index]/48.0f);
                }



                // if (ImGui::Button("Step")) {
                //     board->DataCollectionStart(axis_index + 1, collect_cb);
                //     // board->SetMotorCurrent(axis_index, 0x8100);
                //     Port->WriteAllBoards();
                //     // Amp1394_Sleep(0.100);
                //     // board->SetMotorCurrent(axis_index, 0x8000);
                //     // board->DataCollectionStop();
                //     // Port->WriteAllBoards();
                // }

                ImGui::EndTable();
                ImGui::TreePop();
            }
            // ImGui::Text("%.04f A", im);
            // ImGui::EndChild();
            // ImGui::SameLine();
        }

        ImGui::End();
        Port->WriteAllBoards();

        draw_poke_window();
        // mtx.unlock();
        ImGui::Begin("Tuning");
        static ImPlotFlags flags = ImPlotFlags_NoLegend;
        static ImPlotAxisFlags xflags = ImPlotAxisFlags_AutoFit|ImPlotAxisFlags_NoGridLines;
        static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit|ImPlotAxisFlags_NoGridLines;
        if (ImPlot::BeginPlot("##Tuning","t","I",ImVec2(-1,-1),flags,xflags,yflags)) {
            ImPlot::PlotLine("I measured", current_history_t.data(), current_history.data(), tuning_pulse_width + 200);
            // ImPlot::PlotLine("My Line Plot", x_data, y_data, 1000);
            ImPlot::EndPlot();
        }
        ImGui::End();
        static ImPlotFlags flags2 = ImPlotFlags_NoLegend;
        static ImPlotAxisFlags xflags2 = ImPlotAxisFlags_AutoFit;
        static ImPlotAxisFlags yflags2 = ImPlotAxisFlags_AutoFit;        
        ImGui::Begin("Plot");
        if (ImPlot::BeginPlot("##Plot","t","count",ImVec2(-1,-1),flags,xflags,yflags)) {
            ImPlot::PlotLine("data", plot_y, 1000);
            // ImPlot::PlotLine("My Line Plot", x_data, y_data, 1000);
            ImPlot::EndPlot();
        }
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

    }
    board->WritePowerEnable(false);

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
