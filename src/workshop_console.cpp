#include "workshop_runtime.hpp"
#include "fes_patterns.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>

// ============================================================================
// BOX 3 - TERMINAL USER INTERFACE
// The command sequence and STOP/EXIT handshake are intentionally unchanged.
// ============================================================================
void print_menu()
{

    std::cout << "\n========== EXO-FES WORKSHOP ==========\n";

    std::cout << "1 - ENABLE EXO\n";
    std::cout << "2 - GO TO POSITION\n";
    std::cout << "3 - ENABLE FES\n";

    std::cout << "\n--- Stimulation Pattern ---\n";
    std::cout << "4 - RECTANGULAR\n";
    std::cout << "5 - TRAPEZOIDAL\n";
    std::cout << "6 - BIOMIMETIC\n";

    std::cout << "\n--- Exercise ---\n";
    std::cout << "7 - START EXERCISE\n";
    std::cout << "8 - STOP EXO + FES (3 reinitializes FES; 2 -> 7 restarts Exo)\n";

    std::cout << "\n--- Safe Exit ---\n";
    std::cout << "\n9 - EXIT\n";

    std::cout << "\n \n \nSPACE + ENTER - SHOW COMMAND LIST\n";

    std::cout << "======================================\n";
}

void terminal_input()
{

    std::string input;
    int selection;

    while (true)
    {

        if (!std::getline(std::cin, input))
        {
            break;
        }

        // SPACE + ENTER -> print menu
        if (!input.empty() &&
            input.find_first_not_of(" \t") == std::string::npos)
        {
            print_menu();
            continue;
        }

        std::stringstream ss(input);
        char extra;

        if (!(ss >> selection) || (ss >> extra))
        {
            std::cout << "Invalid input. Enter a number.\n";
            continue;
        }

        switch (selection)
        {

        case RECTANGULAR:
            configure_fes_pattern(FesPatternType::RECTANGULAR);
            break;

        case TRAPEZOIDAL:
            configure_fes_pattern(FesPatternType::TRAPEZOIDAL);
            break;

        case BIOMIMETIC:
            configure_fes_pattern(FesPatternType::BIOMIMETIC);
            break;

        case ENABLE_EXO:
        case GO_TO_POSITION:
        case ENABLE_FES:
        case START_EXERCISE:

            requested_button.store(
                static_cast<GUI_State>(selection));
            break;

        case STOP_EXERCISE:

            stop_requested = true;
            break;

        case EXIT:

            if (!stop_confirmed.load())
            {
                std::cout << "[WARNING] Stop Exo first (command 8).\n";
            }
            else
            {
                requested_button.store(EXIT);

                // Wait here, not in the control callback. loop() must first
                // acknowledge that the session cannot resume or use the logger.
                while (!logger_close_ready.load())
                    usleep(10000);

                if (logger != nullptr)
                {
                    logger.reset();
                    std::cout << "[LOGGER] CSV closed.\n";
                }

                // The callback has acknowledged closure and cannot access P24 again.
                if (P24_port_open)
                {
                    if (smpt_close_serial_port(&p24_stimulator.stim_device))
                    {
                        P24_port_open = false;
                        std::cout << "[FES] Serial port closed.\n";
                    }
                    else
                    {
                        std::cerr << "[FES] Serial port close failed; command 9 retries.\n";
                        break;
                    }
                }

                std::cout << "[SYSTEM] Session closed. Press Ctrl+C to terminate.\n";
            }

            break;

        default:
            std::cout << "Unknown command.\n";
            break;
        }
    }
}
