#pragma once
#include "api.h"    // IWYU pragma: keep

namespace dr4b
{
    // Call once at startup (in initialize()) so Hold() can just brake().
    // Sets the lift and wrist brake modes to HOLD a single time.
    void init();

    namespace Wrist
    {
        void CW();
        void CCW();
        void Hold();
        void Ctr();
    }

    namespace Claw
    {
        void Open();
        void Close();
        void Ctr();
    }

    namespace Lifter
    {
        void Up();
        void Down();
        void Hold();
        void Ctr();
    }

    void Ctr();
}