#include <cmath>
#include <thread>
#include <chrono>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <future>
#include <iostream>
#include <sys/wait.h>

#define PI 3.14159265358979323846

using namespace mavsdk;
using namespace std;
using namespace std::chrono;

// 코사인, 사인 세타값 구하기
typedef double custem_t;
custem_t degreetoradian(custem_t degree) { return((PI / 180.0) * degree); }

int main(int argc, char** argv) {
// connect_result @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    Mavsdk mavsdk; ConnectionResult connection_result;
    bool discovered_system = false;
    connection_result = mavsdk.add_any_connection(argv[1]);
    if (connection_result != ConnectionResult::Success) { return 1; } // Failed
        mavsdk.subscribe_on_new_system([&mavsdk, &discovered_system]() {
        const auto system = mavsdk.systems().at(0);
        if (system->is_connected()) { discovered_system = true; } //Discovered system
        }); // Waiting to discover system...
    std::this_thread::sleep_for(std::chrono::seconds(2)); // heartbeats at 1Hz
    if (!discovered_system) { return 1; } // No system found
    const auto system = mavsdk.systems().at(0);
    system->register_component_discovered_callback(
        [](ComponentType component_type) -> void { std::cout << unsigned(component_type) ; }
    ); // Register a callback components (camera, gimbal) etc are found

// Regist_Telemetry @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    auto telemetry = std::make_shared<Telemetry>(system);
    {
        // We want to listen to the altitude of the drone at 1 Hz.
        const Telemetry::Result set_rate_result = telemetry->set_rate_position(1.0);
        if (set_rate_result != Telemetry::Result::Success) { return 1; } // Set rate failed
        telemetry->subscribe_position([](Telemetry::Position position) {
            std::cout << "Altitude: " << position.relative_altitude_m << " m" << std::endl;
        }); // Set up callback to monitor altitude
        while (telemetry->health_all_ok() != true) {
            std::cout << "Vehicle is getting ready to arm" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } // Check if vehicle is ready to arm
    }

// Arm @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    auto action = std::make_shared<Action>(system);
    {
        
        std::cout << "Arming..." << std::endl;
        const Action::Result arm_result = action->arm();
        if (arm_result != Action::Result::Success) {
            std::cout << "Arming failed:" << arm_result << std::endl;
            return 1;
        }
    }

// Takeoff @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    {
        std::cout << "Taking off..." << std::endl;
        const Action::Result takeoff_result = action->takeoff();
        if (takeoff_result != Action::Result::Success) {
            std::cout << "Takeoff failed:" << takeoff_result << std::endl;
            return 1;
        }
    }

// Upload Mission(1) @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    // next telemetry→healty_all_ok
    auto mission = std::make_shared<Mission>(system);
    {
                
        
        std::vector<Mission::MissionItem> mission_items;
        Mission::MissionItem mission_item;
        mission_item.latitude_deg = 47.3977514; // range: -90 to +90
        mission_item.longitude_deg = 8.54560709; // range: -180 to +180

        mission_item.relative_altitude_m = 10.0f; // takeoff altitude
        mission_item.speed_m_s = 10.0f;
        mission_item.is_fly_through = true; // stop on the waypoint
        mission_items.push_back(mission_item);

        // mission_item.latitude_deg = 47.398170327054473; // range: -90 to +90
        // mission_item.longitude_deg = 8.5456490218639658; // range: -180 to +180
        custem_t distance = 0.000005;
        custem_t radian;
        custem_t target_x;
        custem_t target_y;


        // nDeg : degree 값
        int nDeg = 0;
        for (nDeg = 0; nDeg < 360; nDeg++)
        {
            radian = degreetoradian(nDeg);
            target_x = mission_item.latitude_deg + distance * cos(radian);
            target_y = mission_item.longitude_deg + distance * sin(radian);

            mission_item.latitude_deg = target_x;
            mission_item.longitude_deg = target_y;
            mission_items.push_back(mission_item);

            target_x = 0;
            target_y = 0;
        }
        
// Upload Mission(2) @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        auto prom = std::make_shared<promise<Mission::Result>>();
        auto future_result = prom->get_future();
        Mission::MissionPlan mission_plan;
        mission_plan.mission_items = mission_items;
        mission->upload_mission_async(mission_plan,[prom](Mission::Result result) {prom->set_value(result);});
        const Mission::Result result = future_result.get();
        if (result != Mission::Result::Success) { return 1; }
    }
// Mission Progress @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    {
       std::cout << "Starting mission." << std::endl;
        auto start_prom = std::make_shared<promise<Mission::Result>>();
        auto future_result = start_prom->get_future();
        mission->start_mission_async([start_prom](Mission::Result result) {
            start_prom->set_value(result);
            std::cout << "Started mission." << std::endl;
        });
        const Mission::Result result = future_result.get();
        if (result != Mission::Result::Success) { return -1; } // Mission start failed
    } 
    while (!mission->is_mission_finished().second) {
        std::this_thread::sleep_for(std::chrono::seconds(1)); // Not finished mission.
        }
// Land @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    {
        std::this_thread::sleep_for(std::chrono::seconds(3)); // stuck air
        std::cout << "Landing..." << std::endl;
        const Action::Result land_result = action->land();
        if (land_result != Action::Result::Success) { return 1; } 
        while (telemetry->in_air()) { // Check if vehicle is still in air
            std::this_thread::sleep_for(std::chrono::seconds(1)); // Vehicle is landing...
        }
        std::cout << "Landed!" << std::endl;
        // Relying on auto-disarming but let's keep watching the telemetry for a bit longer.
        std::this_thread::sleep_for(std::chrono::seconds(3));
        std::cout << "Finished..." << std::endl;
    }
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    return 0;
    }

//~/build$ ./connect_result udp://:14540 
