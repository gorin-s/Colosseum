// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_TemaCopter_hpp
#define msr_airlib_vehicles_TemaCopter_hpp

#include "vehicles/multirotor/firmwares/arducopter/ArduCopterApi.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"

namespace msr
{
namespace airlib
{

    class TemaCopterParams : public MultiRotorParams
    {
    public:
        TemaCopterParams(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting, std::shared_ptr<const SensorFactory> sensor_factory)
            : sensor_factory_(sensor_factory)
        {
            connection_info_ = getConnectionInfo(vehicle_setting);
        }

        virtual ~TemaCopterParams() = default;

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

            params.rotor_params.propeller_diameter = 0.1778f; //7"
            params.rotor_params.propeller_height = 0.0072f; //7.2 mm
            params.rotor_params.max_rpm = 20000.0f; //Approx 20000 RPM on ECOII 2807 

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
