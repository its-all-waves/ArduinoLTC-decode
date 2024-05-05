
// #define MAX_EVENTS 6

// #define DEBUG(MSG) Serial.println(MSG)

// // Event Data Types
// typedef char LTC[12]; // timecode or userbits string

// #include "TCDisplayController.h"

// typedef DFRobot_8DigSegDisplController TCDisplayController;

// class StateMachine { };

// class Dispatcher {
// public:
//     Dispatcher(TCDisplayController& tc_displ_controller)
//         : tc_display_controller(tc_displ_controller)
//     {
//     }

// private:
//     TCDisplayController tc_display_controller;

//     /* args communicate who gets wired into the event system */
//     void on_opened_clapper()
//     {
//         DEBUG("OPENED CLAPPER");

//         tc_display_controller.display_off();
//     }

//     void on_closed_clapper(LTC timecode)
//     {
//         DEBUG("CLOSED CLAPPER");

//         tc_display_controller.freeze_display(timecode);
//         //
//     }

//     void on_read_new_frame()
//     {
//         DEBUG("READ NEW FRAME");

//         tc_display_controller.
//     }
// };