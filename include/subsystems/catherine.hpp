#pragma once
#include "api.h"    // IWYU pragma: keep

namespace catherine 
{
    // Call once at startup (in initialize()) so Hold() can just brake().
    // Sets the lift and wrist brake modes to HOLD a single time.
    void init();

    namespace Intake
    {
        void INtake();
        void outake();
        void Hold();
        void Ctr();
    }

    namespace Claw
    {
        void Open();
        void Close();
        void Ctr();
    }

    namespace Wrist
    {
        void Normal();
        void Flipped();
        void Hold();
        void Ctr();
    }

    namespace Cascade
    {
        void Up();
        void Down();
        void Hold();
        void Ctr();
    }

    void Ctr();
}