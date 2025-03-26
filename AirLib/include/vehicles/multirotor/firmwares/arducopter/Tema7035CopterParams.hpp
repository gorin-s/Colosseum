// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_Tema7035Copter_hpp
#define msr_airlib_vehicles_Tema7035Copter_hpp

#include "vehicles/multirotor/firmwares/arducopter/ArduCopterApi.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"

namespace msr
{
namespace airlib
{

    class Tema7035CopterParams : public MultiRotorParams
    {
    public:
        Tema7035CopterParams(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting, std::shared_ptr<const SensorFactory> sensor_factory)
            : sensor_factory_(sensor_factory)
        {
            connection_info_ = getConnectionInfo(vehicle_setting);
        }

        virtual ~Tema7035CopterParams() = default;

        virtual std::unique_ptr<MultirotorApiBase> createMultirotorApi() override
        {
            return std::unique_ptr<MultirotorApiBase>(new ArduCopterApi(this, connection_info_));
        }

    protected:
        virtual void setupParams() override
        {
            auto& params = getParams();

            // Use connection_info_.model for the model name, see Px4MultiRotorParams for example

            setupTema(params);
            //setupFrameGenericQuad(params);
        }

        virtual const SensorFactory* getSensorFactory() const override
        {
            return sensor_factory_.get();
        }

        static const AirSimSettings::MavLinkConnectionInfo& getConnectionInfo(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting)
        {
            return vehicle_setting.connection_info;
        }

    private:
        void setupTema(Params& params) {
            params.rotor_count = 4;
            std::vector<real_T> arm_lengths(params.rotor_count, 0.1475f); //Typical Mark4 7" frame have 295 mm arms, taking one arm as 147.5 mm
            params.mass = 1.6f; //Total mass with battery about 1.6kg
            real_T motor_assembly_weight = 0.060f; //ECOII 2807 + 7037 prop - about 60 grams
            real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

            params.rotor_params.propeller_diameter = 0.1788f; //7"
            params.rotor_params.propeller_height = 0.0070f; //7.0 mm
            params.rotor_params.max_rpm = 13461.5f; //Approx 20000 RPM on ECOII 2807 

            /*
              https://www.youtube.com/watch?v=GAydIsxsKAQ
              https://ekran.store/content/images/10/737x336l80mc0/15191208475239.webp

              Based on similar motor we can adjust C_T to about 0.1637, so that calculated thrust is similar to one, provided in the table
              |Throttle, %                        |   30|   40|    50|    60|    70|    80|    90|    100|
              |Voltage, V                         |   24|   24|    24|    24|    24|    24|    24|     24|
              |Current, A                         |    3|  5,2|     9|  13,6|  19,2|  27,1|    36|     42|
              |RPM                                | 8569|10635| 12323| 15062| 16816| 19185| 20539|  21523|
              |Thrust, gF                         |  407|  593|   892|  1164|  1466|  1932|  2301|   2538|
              |Power, W                           |   72|124,8|   216| 326,4|458,88|647,69| 856,8|1000,14|
              |Thrust, based on C_T, N            |4,088|6,296| 8,454|12,629|15,742|20,490|23,484| 25,788| With C_T = 0.1637 Thrusts are similar
              |Torque, based on C_P, Nm           |0,049|0,076| 0,102| 0,152| 0,190| 0,247| 0,283|  0,311| IDK what it means, kinda similar to torque in youtube video, 
              |                                   |     |     |      |      |      |      |      |       | but there max torque is 0.35 @ 7200 RPM and min torque is 0.04 @ 15000 RPM
              |Motor/Electric Efficiency, %       |   70|   72|    74|    76|    76|    75|    70|     70| Effiency taken from youtube video
              |Motor Power, based on Efficiency, W| 50,4|89,86|159,84|248,06|348,75|485,77|599,76| 700,10| Electrical Power * Efficiency
              |Power, based on C_P, W             |44,19|84,49|131,44|240,01| 334,0|495,97|608,57| 700,30| With C_P = 0.0697 Power is similar

            */
            params.rotor_params.C_T = 0.1637f;
            params.rotor_params.C_P = 0.0697f;
            params.rotor_params.calculateMaxThrust(); //TODO

            // Dimensions of core body box or abdomen, in meters (not including arms).
            // Approx 215x62x70 mm
            params.body_box.x() = 0.215f;
            params.body_box.y() = 0.062f;
            params.body_box.z() = 0.070f;

            // Meters up from center of box mass - about 25mm to the point, where prop is attached
            // Real magic
            real_T rotor_z = 0.025f;

            //computer rotor poses
            initializeRotorQuadX(params.rotor_poses, params.rotor_count, arm_lengths.data(), rotor_z);

            //compute inertia matrix
            computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);
        }
    
        AirSimSettings::MavLinkConnectionInfo connection_info_;
        std::shared_ptr<const SensorFactory> sensor_factory_;
    };
}
} //namespace
#endif
