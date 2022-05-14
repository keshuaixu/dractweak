#include <Amp1394/AmpIORevision.h>
#include "PortFactory.h"
#include "AmpIO.h"
#include "Amp1394Time.h"
#include "EthBasePort.h"
#include <chrono>
#include <thread>
#include <iomanip>


void sleep(int t) {
    std::this_thread::sleep_for(std::chrono::milliseconds(t));
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

int main(int argc, char** argv)
{
    std::cout << "dRAC factory test" << std::endl;
    std::cout << "Do not attach real robot." << std::endl;

    BasePort::ProtocolType protocol = BasePort::PROTOCOL_SEQ_R_BC_W;
    int board_id = 0;
    auto board = new AmpIO(board_id);
    std::string portDescription = "udp";
    auto Port = PortFactory(portDescription.c_str(), std::cout);
    EthBasePort *ethPort = dynamic_cast<EthBasePort *>(Port);
    Port->AddBoard(board);
    board->WriteWatchdogPeriodInSeconds(0);
    if (Port->GetHardwareVersion(board_id) != 0x64524131) {
        std::cerr << "the board does not appear to be a dRAC" << std::endl;
        std::cerr << "hw ver " << Port->GetHardwareVersion(board_id) << std::endl;
    }


    // lvds
    // relay contact
    // estop sense
    // power up, power down
    // ESPMV
    // amp/adc zero
    // open loop drive
    // closed loop drive
    



    Port->WriteQuadlet(board_id, 0xB100, 1); // bypass safety
    Port->WriteAllBoards();


    test("Safety relay open");
    board->WriteSafetyRelay(0);
    sleep(100);
    if (board->ReadSafetyRelayStatus() == 0) pass(); else fail();

    test("Safety relay close");
    board->WriteSafetyRelay(1);
    sleep(100);
    if (board->ReadSafetyRelayStatus() == 1) pass(); else fail();

    test("48V power off");
    board->SetPowerEnable(0);
    Port->WriteAllBoards();
    sleep(500);
    auto mv_good = board->GetPowerStatus();
    quadlet_t mv;
    Port->ReadQuadlet(board_id, 0xb002, mv);
    if (mv_good == 0) pass(); else fail();

    test("48V power on");
    board->SetPowerEnable(1);
    Port->WriteAllBoards();
    sleep(500);
    mv_good = board->GetPowerStatus();
    Port->ReadQuadlet(board_id, 0xb002, mv);
    if (mv_good == 1) pass(); else fail();
    
    test("ESPM 6V");
    quadlet_t espmv_good;
    Port->ReadQuadlet(board_id, 0xb003, espmv_good);
    if (espmv_good & 0x1 == 0) pass(); else fail();




    // test("Press E Stop...");
    // board->
    // test("Release E Stop...");




    test("LVDS loopback");
    quadlet_t crc_good_count;
    Port->ReadQuadlet(board_id, 0xb001, crc_good_count); 
    Port->WriteQuadlet(board_id,0xB101 , 0x0);
    Port->WriteQuadlet(board_id, 0xB101, 0xAC450F28); 
    for (int i=0; i < 48; i++)
        Port->WriteQuadlet(board_id,0xB101 , 0x12121212);
    Port->WriteQuadlet(board_id,0xB101 , 0x2f1d);     
    quadlet_t crc_good_count2;
    Port->ReadQuadlet(board_id, 0xb001, crc_good_count2);
    if (crc_good_count2 - crc_good_count == 1) pass(); else {
        fail();
        std::cerr << "Did you attach the loopback plug?" << std::endl;
    }

    // sleep(100);

    // for (int index = 0; index < 10; index++){
    //     DYNAMIC_SECTION("PWM mode channel " << index) {
    //         double voltage = 0.01;
    //         board->SetAmpEnable(index, 1);
    //         Port->WriteAllBoards();
    //         board->SetMotorVoltageRatio(index, voltage);
    //         Port->WriteAllBoards();
    //         // Port->ReadAllBoards();
    //         sleep(50);
    //         Port->ReadAllBoards();
    //         auto v_read = board->GetMotorVoltageRatio(index);
    //         auto i_read = board->GetMotorCurrent(index);
    //         board->SetAmpEnable(index, 0);
    //         Port->WriteAllBoards();
    //         REQUIRE(v_read == Catch::Approx(voltage).epsilon(0.05));
    //         REQUIRE(i_read > 32767 + 10);
    //     }
    // }


    // board->SetPowerEnable(0);
    // Port->WriteAllBoards();
}