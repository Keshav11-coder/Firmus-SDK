#ifndef FIRMUS_H
#define FIRMUS_H

/** © 2025 Keshav Haripersad
 *  Base API header - Firmus.h
 *  | Firmus.cpp for partial logic.
 *  | /Firmus for modules/assets.
 *
 *  Licensed under the Apache 2.0 license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/Firmus-SDK
 *  Firmus SDK version: 1.3.3
 */

// Arduino environment
#include <Arduino.h>

// Matrix operations
#include "Firmus/Assets/matrix2.h"

// General Frame Vector (Vector3) implementation
struct Vector3
{
    float x; // X component
    float y; // Y component
    float z; // Z component

    Vector3(float x_ = 0, float y_ = 0, float z_ = 0) : x(x_), y(y_), z(z_) {}
};

// Namespace Firmus
namespace Firmus
{
    /*
     * Actuator Interface class (SHOULD BE MOVED TO SEPARATE MODULE)
     * | Interface for thruster controllers (e.g. ESCs, Boosters)
     * | Provides a failsafe system for emergency situations
     * | Provides, at its minimum, basic control functions
     * | Class is abstract and must be inherited and implemented due to expected implmentations (e.g. failsafe / write)
     * | Functionality is not constrained. It is up to the user to implement the necessary functions
     * | An inherited class can be found in /Firmus: ESC.h
     */
    class ActuatorInterface
    {
    public:
        struct failsafe_system // Failsafe system struct as example
        {
        private:
            bool failsafe = false;

        public:
            virtual bool engage() // Enable failsafe
            {
                failsafe = true;
                return true;
            }

            virtual bool disengage() // Disable failsafe
            {
                failsafe = false;
                return true;
            }

            virtual bool status() // Get failsafe status
            {
                return failsafe;
            }

            virtual ~failsafe_system() = default; // Destructor if needed
        };

        virtual int arm() = 0;              // Thruster arm function (e.g. ESC arming, or Booster initialization)
        virtual int stop() = 0;             // Thruster stop function (e.g. ESC stop, or Booster shutdown)
        virtual int restart() = 0;          // Thruster restart function (e.g. ESC restart, or Booster re-initialization)
        virtual int max() = 0;              // Thruster maximum speed function
        virtual int min() = 0;              // Thruster minimum speed function
        virtual int write(float speed) = 0; // Thruster write function (e.g. ESC write, or Booster write)

        failsafe_system failsafe;
    };
    
    // BENCHED FOR NOW
    // template <typename Model, typename... Modules>
    // class Drone
    // {
    // private:
    //     // To hold the modules
    //     struct AbstractModule
    //     {
    //     };

    //     using ModuleVariant = std::variant<AbstractModule, Modules...>;
    //     std::unordered_map<size_t, ModuleVariant> _modules;
    //     size_t moduleNextId = 0;

    // public:
    //     // Profile accessor
    //     Model *model;

    //     // Basic constructor (just for computation)
    //     Drone() {}

    //     // Add a module of type ModuleType
    //     template <typename ModuleType, typename... Args>
    //     size_t add(Args &&...constructor)
    //     {
    //         size_t id = moduleNextId;
    //         _modules.insert({id, ModuleType(std::forward<Args>(constructor)...)});
    //         moduleNextId++;
    //         return id;
    //     }

    //     // Get a module of type ModuleType by id
    //     template <typename ModuleType>
    //     ModuleType *get(size_t id)
    //     {
    //         auto it = _modules.find(id);
    //         if (it != _modules.end())
    //         {
    //             if (auto module = std::get_if<ModuleType>(&it->second))
    //             {
    //                 return module; // Return pointer to the module if the type matches
    //             }
    //             else
    //             {
    //                 return nullptr; // Type mismatch, return nullptr
    //             }
    //         }
    //         return nullptr; // Return nullptr if the module doesn't exist
    //     }

    //     // Drone constructor now initializes the 'module' struct with proper references
    //     Drone(Model *model_)
    //         : model(model_)
    //     {
    //     }
    // };
}

#endif