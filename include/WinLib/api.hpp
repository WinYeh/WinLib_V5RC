#pragma once

// only include those that the WinLib used 
#include "pros/rtos.hpp"                   // IWYU pragma: keep
#include "WinLib/pid.hpp"                  // IWYU pragma: keep 
#include "WinLib/util.hpp"                 // IWYU pragma: keep
#include "WinLib/timer.hpp"                // IWYU pragma: keep
#include "WinLib/exitcondition.hpp"        // IWYU pragma: keep
#include "WinLib/pose.hpp"                 // IWYU pragma: keep
#include "WinLib/chassis/Odom.hpp"         // IWYU pragma: keep
#include "WinLib/chassis/OdomSensors.hpp"  // IWYU pragma: keep
#include "WinLib/chassis/DSR.hpp"          // IWYU pragma: keep
#include "WinLib/chassis/chassis.hpp"      // IWYU pragma: keep


/*                         _
                        _ooOoo_
                       o8888888o
                       88" . "88
                       (| -_- |)
                       O\  =  /O
                    ____/`---'\____
                  .'  \\|     |//  `.
                 /  \\|||  :  |||//  \
                /  _||||| -:- |||||_  \
                |   | \\\  -  /'| |   |
                | \_|  `\`---'//  |_/ |
                \  .-\__ `-. -'__/-.  /
              ___`. .'  /--.--\  `. .'___
           ."" '<  `.___\_<|>_/___.' _> \"".
          | | :  `- \`. ;`. _/; .'/ /  .' ; |
          \  \ `-.   \_\_`. _.'_/_/  -' _.' /
===========`-.`___`-.__\ \___  /__.-'_.'_.-'================
                        `=--=-'                    
*/